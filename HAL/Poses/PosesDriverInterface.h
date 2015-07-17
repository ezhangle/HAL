#pragma once

#include <functional>
#include <HAL/Devices/DriverInterface.h>
#include <HAL/Pose.pb.h>


namespace hal {

typedef std::function<void (hal::PoseMsg&)> PosesDriverDataCallback;
typedef std::function<void ()> PosesDriverFinishedLoadingDataCallback;



/// Generic Wheel Odometry driver interface
///
class PosesDriverInterface : public DriverInterface {
 public:
  virtual ~PosesDriverInterface() {}
  virtual void RegisterPosesDataCallback(PosesDriverDataCallback callback) = 0;
  virtual void RegisterPosesFinishedLoadingDataCallback(PosesDriverFinishedLoadingDataCallback callback) = 0;
  virtual bool IsRunning() const = 0;

};


} /* namespace */
