#include <chrono>
#include <thread>

#include "io/dm_imu/dm_imu.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  const std::string port = argc > 1 ? argv[1] : "/dev/ttyACM0";
  io::DM_IMU imu(port);

  while (!exiter.exit()) {
    auto timestamp = std::chrono::steady_clock::now();

    std::this_thread::sleep_for(1ms);

    Eigen::Quaterniond q = imu.imu_at(timestamp);

    Eigen::Vector3d eulers = tools::eulers(q, 2, 1, 0) * 57.3;
    tools::logger()->info(
      "w{:.4f} x{:.4f} y{:.4f} z{:.4f}  yaw{:.2f} pitch{:.2f} roll{:.2f}", q.w(), q.x(), q.y(),
      q.z(), eulers[0], eulers[1], eulers[2]);
  }

  return 0;
}