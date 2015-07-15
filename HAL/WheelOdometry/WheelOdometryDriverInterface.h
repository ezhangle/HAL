#pragma once

#include <functional>
#include <HAL/Devices/DriverInterface.h>
#include <HAL/WheelOdometry.pb.h>

namespace hal {

typedef std::function<void (hal::WheelOdometryMsg&)> WheelOdometryDriverDataCallback;

/// Generic Wheel Odometry driver interface
///
class WheelOdometryDriverInterface : public DriverInterface {
 public:
  virtual ~WheelOdometryDriverInterface() {}
  virtual void RegisterWheelOdometryDataCallback(WheelOdometryDriverDataCallback callback) = 0;
  virtual bool IsRunning() const = 0;
};

} /* namespace */
