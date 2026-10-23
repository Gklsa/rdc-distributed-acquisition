// C++ counterpart of rdc_core/experiment_manager_node.py.
//
// Same ROS interface as the Python/subprocess version (services, /experiment/command,
// /experiment/state JSON schema, recordings_dir/<exp_id>/<device>_<ts>/ layout) so the
// rest of the system (pc_master, runtime_status, Makefile, dashboard) does not need to
// change when switching recording_mode. The only difference is how the bag gets written:
// rosbag2_cpp::Writer in-process instead of spawning `ros2 bag record`.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <regex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/storage_options.hpp"

#include <nlohmann/json.hpp>

namespace fs = std::filesystem;
using nlohmann::json;

namespace
{

std::vector<std::string> split_csv(const std::string & s)
{
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ',')) {
    const size_t start = item.find_first_not_of(" \t");
    const size_t end = item.find_last_not_of(" \t");
    if (start != std::string::npos) {
      out.push_back(item.substr(start, end - start + 1));
    }
  }
  return out;
}

std::string zero_pad(int n, int width)
{
  std::ostringstream oss;
  oss << std::setfill('0') << std::setw(width) << n;
  return oss.str();
}

// "2026-05-29T10:00:00Z" / "...+00:00" / no-suffix — always treated as UTC.
std::string iso_now_utc()
{
  const auto t = std::time(nullptr);
  std::tm tm_utc{};
  gmtime_r(&t, &tm_utc);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &tm_utc);
  return std::string(buf) + "+00:00";
}

std::string compact_timestamp_utc()
{
  const auto t = std::time(nullptr);
  std::tm tm_utc{};
  gmtime_r(&t, &tm_utc);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y%m%dT%H%M%SZ", &tm_utc);
  return std::string(buf);
}

// Best-effort ISO8601 UTC parse, ignoring fractional seconds / timezone suffix.
// Only used for the late_join heuristic — falls back to "normal" on any parse failure,
// same as the python implementation's try/except.
bool parse_iso8601_utc(const std::string & s, std::chrono::system_clock::time_point & out)
{
  std::tm tm{};
  std::istringstream ss(s);
  ss >> std::get_time(&tm, "%Y-%m-%dT%H:%M:%S");
  if (ss.fail()) {
    return false;
  }
  out = std::chrono::system_clock::from_time_t(timegm(&tm));
  return true;
}

// Written by `make pi-sync-code` at packaging time (PC-side, real .git). Real Pis have no
// .git of their own (excluded from the sync tarball), so this file is their only source of
// truth for which commit/clean-state was actually deployed; fall back to `git` directly for
// setups where /ros2_ws is a real repo checkout (e.g. the docker-compose simulation).
const char * const kGitSyncInfoPath = "/ros2_ws/.git_sync_info.json";
const char * const kRepoDir = "/ros2_ws";

std::string run_command(const std::string & cmd)
{
  std::array<char, 256> buffer{};
  std::string result;
  FILE * pipe = popen((cmd + " 2>/dev/null").c_str(), "r");
  if (!pipe) {
    return "";
  }
  while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
    result += buffer.data();
  }
  pclose(pipe);
  while (!result.empty() && (result.back() == '\n' || result.back() == '\r')) {
    result.pop_back();
  }
  return result;
}

struct CodeVersion
{
  std::string commit_hash = "unknown";
  bool is_clean = false;
  bool known = false;
};

CodeVersion get_code_version()
{
  if (fs::exists(kGitSyncInfoPath)) {
    std::ifstream f(kGitSyncInfoPath);
    try {
      const json info = json::parse(f);
      return {info.value("commit_hash", "unknown"), info.value("is_clean", false), true};
    } catch (const json::parse_error &) {
      // Fall through to the git fallback below.
    }
  }

  // -c safe.directory=... bypasses git's "dubious ownership" check, which otherwise
  // blocks these commands whenever the bind-mounted repo is owned by a different UID
  // than the container's (e.g. host user vs. root inside the container).
  const std::string git_prefix =
    std::string("git -c safe.directory=") + kRepoDir + " -C " + kRepoDir;
  const std::string hash = run_command(git_prefix + " rev-parse HEAD");
  if (hash.empty()) {
    return {};
  }
  const std::string status = run_command(git_prefix + " status --porcelain");
  return {hash, status.empty(), true};
}

}  // namespace

class ExperimentManagerCpp : public rclcpp::Node
{
public:
  ExperimentManagerCpp()
  : Node("experiment_manager")
  {
    declare_parameter("recordings_dir", std::string("/ros2_ws/recordings"));
    declare_parameter("topics", std::string("/imu/data,/temperature,/camera/image_raw"));
    declare_parameter("device_name", std::string(""));
    declare_parameter("accepted_master_device_id", std::string(""));

    recordings_dir_ = get_parameter("recordings_dir").as_string();
    topics_ = split_csv(get_parameter("topics").as_string());
    accepted_master_device_id_ = get_parameter("accepted_master_device_id").as_string();

    const std::string device_override = get_parameter("device_name").as_string();
    device_id_ = sanitize_device_id(
      device_override.empty() ? default_device_id() : device_override);

    fs::create_directories(recordings_dir_);

    create_recording_subscriptions();

    start_srv_ = create_service<std_srvs::srv::Trigger>(
      "/" + device_id_ + "/start_experiment",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        const auto [ok, msg] = do_start("", "", "");
        response->success = ok;
        response->message = msg;
      });

    stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "/" + device_id_ + "/stop_experiment",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        const auto [ok, msg] = do_stop();
        response->success = ok;
        response->message = msg;
      });

    cmd_sub_ = create_subscription<std_msgs::msg::String>(
      "/experiment/command", 10,
      std::bind(&ExperimentManagerCpp::command_callback, this, std::placeholders::_1));

    state_pub_ = create_publisher<std_msgs::msg::String>("/experiment/state", 10);

    RCLCPP_INFO(
      get_logger(), "ExperimentManager (C++ / rosbag2_cpp::Writer) ready. device_id=%s",
      device_id_.c_str());
    RCLCPP_INFO(get_logger(), "  recordings_dir : %s", recordings_dir_.c_str());
    RCLCPP_INFO(
      get_logger(), "  accepted_master: %s",
      accepted_master_device_id_.empty() ? "any" : accepted_master_device_id_.c_str());
    for (const auto & t : topics_) {
      RCLCPP_INFO(get_logger(), "  topic          : %s", t.c_str());
    }
  }

private:
  // ------------------------------------------------------------------
  // Topic registry — rosbag2_cpp::Writer::write<T>() needs the message
  // type at compile time, so (unlike `ros2 bag record`) we can only
  // record topics we know the type of ahead of time.
  // ------------------------------------------------------------------
  template<typename MessageT>
  void add_recording_subscription(const std::string & topic)
  {
    auto sub = create_subscription<MessageT>(
      topic, 10,
      [this, topic](const typename MessageT::SharedPtr msg) {
        if (writer_) {
          writer_->write(*msg, topic, now());
        }
      });
    subscriptions_.push_back(sub);
  }

  void create_recording_subscriptions()
  {
    for (const auto & topic : topics_) {
      if (topic == "/temperature") {
        add_recording_subscription<sensor_msgs::msg::Temperature>(topic);
      } else if (topic == "/imu/data") {
        add_recording_subscription<sensor_msgs::msg::Imu>(topic);
      } else if (topic == "/camera/image_raw") {
        add_recording_subscription<sensor_msgs::msg::Image>(topic);
      } else {
        RCLCPP_WARN(
          get_logger(),
          "No known message type for topic '%s' -- experiment_manager_cpp will not record it "
          "(add it to create_recording_subscriptions() in experiment_manager_cpp_node.cpp)",
          topic.c_str());
      }
    }
  }

  // ------------------------------------------------------------------
  // Command handling (mirrors experiment_manager_node.py)
  // ------------------------------------------------------------------
  void command_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    json data;
    try {
      data = json::parse(msg->data);
    } catch (const json::parse_error &) {
      RCLCPP_ERROR(get_logger(), "Invalid JSON on /experiment/command: %s", msg->data.c_str());
      return;
    }

    std::string cmd = data.value("command", "");
    std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::toupper);
    const std::string ext_id = data.value("experiment_id", "");
    const std::string pc_ts = data.value("pc_timestamp", "");
    const std::string source_master = data.value("source_device_id", "");

    if (!accepted_master_device_id_.empty() && source_master != accepted_master_device_id_) {
      RCLCPP_WARN(
        get_logger(),
        "Ignoring %s for %s: source_device_id=%s does not match accepted_master_device_id=%s",
        cmd.c_str(), ext_id.c_str(), source_master.c_str(), accepted_master_device_id_.c_str());
      return;
    }

    if (cmd == "START") {
      if (writer_ && exp_id_ == ext_id) {
        return;  // heartbeat while already recording this experiment_id -- ignore
      }
      const auto [ok, text] = do_start(ext_id, pc_ts, source_master);
      (void)ok;
      RCLCPP_INFO(get_logger(), "[remote] START -> %s", text.c_str());
    } else if (cmd == "STOP") {
      const auto [ok, text] = do_stop();
      (void)ok;
      RCLCPP_INFO(get_logger(), "[remote] STOP  -> %s", text.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Unknown command: %s", cmd.c_str());
    }
  }

  // ------------------------------------------------------------------
  // Core start/stop logic
  // ------------------------------------------------------------------
  std::pair<bool, std::string> do_start(
    const std::string & experiment_id,
    const std::string & pc_timestamp,
    const std::string & source_master_device_id)
  {
    if (writer_) {
      return {false, "Already recording: " + exp_id_};
    }

    start_time_ = std::chrono::system_clock::now();
    pi_start_time_str_ = iso_now_utc();
    pc_start_time_ = pc_timestamp;
    source_master_device_id_ = source_master_device_id;
    code_version_ = get_code_version();

    if (!experiment_id.empty()) {
      exp_id_ = experiment_id;
    } else {
      exp_id_ = "exp_" + zero_pad(next_exp_number(), 3) + "_" + compact_timestamp_utc();
    }

    const std::string pi_ts = compact_timestamp_utc();
    exp_dir_ = recordings_dir_ + "/" + exp_id_ + "/" + device_id_ + "_" + pi_ts;
    fs::create_directories(exp_dir_);
    bag_path_ = exp_dir_ + "/bag";

    join_mode_ = "normal";
    if (!experiment_id.empty() && !pc_start_time_.empty()) {
      std::chrono::system_clock::time_point pc_tp;
      if (parse_iso8601_utc(pc_start_time_, pc_tp)) {
        const double delay = std::chrono::duration<double>(start_time_ - pc_tp).count();
        if (delay > 15.0) {
          join_mode_ = "late_join";
        }
      }
    }

    json exp_info = {
      {"experiment_id", exp_id_},
      {"device_id", device_id_},
      {"pc_start_time", pc_start_time_.empty() ? json(nullptr) : json(pc_start_time_)},
      {"source_master_device_id",
        source_master_device_id_.empty() ? json(nullptr) : json(source_master_device_id_)},
      {"pi_start_time", pi_start_time_str_},
      {"topics", topics_},
      {"storage_format", "mcap"},
      {"bag_directory", bag_path_},
      {"trigger_mode", experiment_id.empty() ? "local" : "remote"},
      {"join_mode", join_mode_},
      {"code_commit_hash", code_version_.commit_hash},
      {"code_is_clean", code_version_.known ? json(code_version_.is_clean) : json(nullptr)},
    };
    write_json(exp_dir_ + "/experiment_info.json", exp_info);

    open_writer();

    const std::string msg =
      "Started " + exp_id_ + " | device=" + device_id_ + " | dir=" + exp_dir_;
    RCLCPP_INFO(get_logger(), "%s", msg.c_str());
    publish_state();
    return {true, msg};
  }

  std::pair<bool, std::string> do_stop()
  {
    if (!writer_) {
      return {false, "No experiment is currently running"};
    }

    close_writer();

    const auto end_time = std::chrono::system_clock::now();
    const double duration = std::chrono::duration<double>(end_time - start_time_).count();

    json metadata = {
      {"experiment_id", exp_id_},
      {"device_id", device_id_},
      {"pc_start_time", pc_start_time_.empty() ? json(nullptr) : json(pc_start_time_)},
      {"source_master_device_id",
        source_master_device_id_.empty() ? json(nullptr) : json(source_master_device_id_)},
      {"pi_start_time", pi_start_time_str_},
      {"pi_end_time", iso_now_utc()},
      {"duration_seconds", std::round(duration * 1000.0) / 1000.0},
      {"status", "completed"},
      {"topics_recorded", topics_},
      {"bag_directory", bag_path_},
      {"code_commit_hash", code_version_.commit_hash},
      {"code_is_clean", code_version_.known ? json(code_version_.is_clean) : json(nullptr)},
    };
    write_json(exp_dir_ + "/metadata.json", metadata);

    const std::string exp_id_for_msg = exp_id_;
    char duration_buf[32];
    std::snprintf(duration_buf, sizeof(duration_buf), "%.1f", duration);
    const std::string msg = "Stopped " + exp_id_for_msg + " | duration=" + duration_buf + "s";
    RCLCPP_INFO(get_logger(), "%s", msg.c_str());

    // Matches experiment_manager_node.py: join_mode is intentionally left as-is here.
    exp_id_.clear();
    exp_dir_.clear();
    bag_path_.clear();
    pi_start_time_str_.clear();
    pc_start_time_.clear();
    source_master_device_id_.clear();
    code_version_ = {};

    publish_state();
    return {true, msg};
  }

  // ------------------------------------------------------------------
  void open_writer()
  {
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_path_;
    storage_options.storage_id = "mcap";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open(storage_options, converter_options);
  }

  void close_writer()
  {
    writer_.reset();  // Writer's destructor flushes and closes the bag.
  }

  void publish_state()
  {
    json state = {
      {"device_id", device_id_},
      {"experiment_id", exp_id_.empty() ? json(nullptr) : json(exp_id_)},
      {"source_master_device_id",
        source_master_device_id_.empty() ? json(nullptr) : json(source_master_device_id_)},
      {"status", writer_ ? "running" : "idle"},
      {"pi_start_time", pi_start_time_str_.empty() ? json(nullptr) : json(pi_start_time_str_)},
      {"topics", topics_},
      {"bag_directory", bag_path_.empty() ? json(nullptr) : json(bag_path_)},
      {"join_mode", join_mode_.empty() ? json(nullptr) : json(join_mode_)},
    };
    std_msgs::msg::String out;
    out.data = state.dump();
    state_pub_->publish(out);
  }

  std::string sanitize_device_id(const std::string & value)
  {
    std::string safe = std::regex_replace(value, std::regex("[^A-Za-z0-9_]+"), "_");
    safe = std::regex_replace(safe, std::regex("^_+|_+$"), "");
    return safe.empty() ? "device" : safe;
  }

  std::string default_device_id()
  {
    char hostname[256] = {0};
    ::gethostname(hostname, sizeof(hostname) - 1);
    return std::string(hostname);
  }

  int next_exp_number()
  {
    int max_n = 0;
    if (!fs::exists(recordings_dir_)) {
      return 1;
    }
    static const std::regex pattern(R"(^exp_(\d+)_)");
    for (const auto & entry : fs::directory_iterator(recordings_dir_)) {
      std::smatch m;
      const std::string name = entry.path().filename().string();
      if (std::regex_search(name, m, pattern)) {
        max_n = std::max(max_n, std::stoi(m[1].str()));
      }
    }
    return max_n + 1;
  }

  void write_json(const std::string & path, const json & data)
  {
    std::ofstream f(path);
    f << data.dump(2);
    RCLCPP_INFO(get_logger(), "Written: %s", path.c_str());
  }

  // ------------------------------------------------------------------
  std::string recordings_dir_;
  std::vector<std::string> topics_;
  std::string device_id_;
  std::string accepted_master_device_id_;

  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::string exp_id_;
  std::string exp_dir_;
  std::string bag_path_;
  std::string join_mode_;
  std::string pc_start_time_;
  std::string source_master_device_id_;
  std::string pi_start_time_str_;
  std::chrono::system_clock::time_point start_time_;
  CodeVersion code_version_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExperimentManagerCpp>());
  rclcpp::shutdown();
  return 0;
}
