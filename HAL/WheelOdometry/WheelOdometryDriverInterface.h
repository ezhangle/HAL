#pragma once

#include <functional>
#include <HAL/Devices/DriverInterface.h>
#include <HAL/WheelOdometry.pb.h>
#include <HAL/Messages/Pose.h>

namespace hal {

typedef std::function<void (hal::WheelOdometryMsg&)> WheelOdometryDriverDataCallback;
typedef std::function<void ()> WheelOdometryDriverFinishedLoadingDataCallback;



/// Generic Wheel Odometry driver interface
///
class WheelOdometryDriverInterface : public DriverInterface {
 public:
  virtual ~WheelOdometryDriverInterface() {}
  virtual void RegisterWheelOdometryDataCallback(WheelOdometryDriverDataCallback callback) = 0;
  virtual void RegisterWheelOdometryFinishedLoadingDataCallback(WheelOdometryDriverFinishedLoadingDataCallback callback) = 0;
  virtual Sophus::SE3Group<double> IntegrateWheelSpeed(const double start_time,
                                                const std::vector<WheelOdometryMsg>& measurements,
                                                const double end_time) = 0;
  virtual std::vector<hal::PoseMsg> IntegrateWheelSpeed(
                            const std::vector<WheelOdometryMsg>& measurements) = 0;
  virtual bool IsRunning() const = 0;

};


} /* namespace */
