#pragma once

#include <HAL/Devices/SharedLoad.h>

#include <HAL/WheelOdometry/WheelOdometryDriverInterface.h>
#include <HAL/Devices/DeviceFactory.h>
#include <HAL/Utils/Uri.h>

namespace hal {

// Generic WheelOdometry
class WheelOdometry : public WheelOdometryDriverInterface {
 public:
  WheelOdometry() {}

  WheelOdometry(const std::string& uri) : m_URI(uri) {
    m_WheelOdometry = DeviceRegistry<WheelOdometryDriverInterface>::I().Create(m_URI, "WheelOdometry");
  }

  ~WheelOdometry() {
    Clear();
  }

  inline void Reset() {
    Clear();
    m_WheelOdometry = DeviceRegistry<WheelOdometryDriverInterface>::I().Create(m_URI, "WheelOdometry");
    RegisterWheelOdometryDataCallback(m_callback);
  }

  void Clear() {
    m_WheelOdometry = nullptr;
  }

  void RegisterWheelOdometryDataCallback(WheelOdometryDriverDataCallback callback) {
    m_callback = callback;
    if( m_WheelOdometry ){
      m_WheelOdometry->RegisterWheelOdometryDataCallback( callback );
    }else{
      std::cerr << "ERROR: no driver initialized!\n";
    }
    return;
  }


  std::string GetDeviceProperty(const std::string& sProperty) {
    return m_WheelOdometry->GetDeviceProperty(sProperty);
  }

  bool IsRunning() const override {
    return m_WheelOdometry->IsRunning();
  }


 protected:
  hal::Uri                                          m_URI;
  std::shared_ptr<WheelOdometryDriverInterface>     m_WheelOdometry;
  WheelOdometryDriverDataCallback                   m_callback;
};
} /* namespace hal */
