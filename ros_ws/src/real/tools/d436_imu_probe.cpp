#include <librealsense2/rs.hpp>

#include <atomic>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

namespace
{
struct MotionSample
{
  rs2_vector value{};
  double timestamp_ms = 0.0;
  bool valid = false;
};

template<typename InfoSource>
const char * safe_info(const InfoSource & source, rs2_camera_info field)
{
  return source.supports(field) ? source.get_info(field) : "unknown";
}
}  // namespace

int main()
{
  try {
    rs2::context context;
    const auto devices = context.query_devices();
    if (devices.size() == 0) {
      std::cerr << "FAIL: no RealSense device found\n";
      return 2;
    }

    const rs2::device device = devices.front();
    const std::string serial = safe_info(device, RS2_CAMERA_INFO_SERIAL_NUMBER);
    std::cout << "Device: " << safe_info(device, RS2_CAMERA_INFO_NAME)
              << "  serial=" << serial
              << "  firmware=" << safe_info(device, RS2_CAMERA_INFO_FIRMWARE_VERSION)
              << '\n';

    int accel_profiles = 0;
    int gyro_profiles = 0;
    for (const auto & sensor : device.query_sensors()) {
      std::cout << "Sensor: " << safe_info(sensor, RS2_CAMERA_INFO_NAME) << '\n';
      for (const auto & profile : sensor.get_stream_profiles()) {
        const auto stream = profile.stream_type();
        if (stream != RS2_STREAM_ACCEL && stream != RS2_STREAM_GYRO) {
          continue;
        }
        const auto motion = profile.as<rs2::motion_stream_profile>();
        std::cout << "  " << rs2_stream_to_string(stream)
                  << " format=" << rs2_format_to_string(profile.format())
                  << " rate=" << profile.fps() << " Hz\n";
        accel_profiles += stream == RS2_STREAM_ACCEL;
        gyro_profiles += stream == RS2_STREAM_GYRO;
      }
    }

    if (accel_profiles == 0 || gyro_profiles == 0) {
      std::cerr << "FAIL: motion profiles missing (accel=" << accel_profiles
                << ", gyro=" << gyro_profiles << ")\n";
      return 3;
    }

    std::atomic<unsigned long> accel_count{0};
    std::atomic<unsigned long> gyro_count{0};
    MotionSample last_accel;
    MotionSample last_gyro;
    std::mutex sample_mutex;

    rs2::config config;
    config.enable_device(serial);
    config.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    config.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

    rs2::pipeline pipeline(context);
    pipeline.start(config, [&](const rs2::frame & frame) {
      const auto motion = frame.as<rs2::motion_frame>();
      if (!motion) {
        return;
      }

      MotionSample sample;
      sample.value = motion.get_motion_data();
      sample.timestamp_ms = motion.get_timestamp();
      sample.valid = true;

      std::lock_guard<std::mutex> lock(sample_mutex);
      if (motion.get_profile().stream_type() == RS2_STREAM_ACCEL) {
        last_accel = sample;
        ++accel_count;
      } else if (motion.get_profile().stream_type() == RS2_STREAM_GYRO) {
        last_gyro = sample;
        ++gyro_count;
      }
    });

    constexpr auto sample_time = std::chrono::seconds(6);
    const auto start_time = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(sample_time);
    pipeline.stop();
    const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();

    std::lock_guard<std::mutex> lock(sample_mutex);
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Accel: " << accel_count << " frames (" << accel_count / elapsed
              << " Hz), last=[" << last_accel.value.x << ", " << last_accel.value.y
              << ", " << last_accel.value.z << "] m/s^2\n";
    std::cout << "Gyro:  " << gyro_count << " frames (" << gyro_count / elapsed
              << " Hz), last=[" << last_gyro.value.x << ", " << last_gyro.value.y
              << ", " << last_gyro.value.z << "] rad/s\n";

    if (!last_accel.valid || !last_gyro.valid || accel_count == 0 || gyro_count == 0) {
      std::cerr << "FAIL: motion streams opened but did not deliver both frame types\n";
      return 4;
    }

    std::cout << "PASS: D436 accelerometer and gyroscope both delivered data\n";
    return 0;
  } catch (const rs2::error & error) {
    std::cerr << "RealSense error in " << error.get_failed_function() << ": "
              << error.what() << '\n';
    return 5;
  } catch (const std::exception & error) {
    std::cerr << "Error: " << error.what() << '\n';
    return 6;
  }
}
