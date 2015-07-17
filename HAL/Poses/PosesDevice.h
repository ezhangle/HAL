#pragma once

#include <HAL/Devices/SharedLoad.h>

#include <HAL/Poses/PosesDriverInterface.h>
#include <HAL/Devices/DeviceFactory.h>
#include <HAL/Utils/Uri.h>

namespace hal {

// Generic Poses
class Poses : public PosesDriverInterface {
 public:
  Poses() {}

  Poses(const std::string& uri) : m_URI(uri) {
    m_Poses = DeviceRegistry<PosesDriverInterface>::I().Create(m_URI, "Poses");
  }

  ~Poses() {
    Clear();
  }

  inline void Reset() {
    Clear();
    m_Poses = DeviceRegistry<PosesDriverInterface>::I().Create(m_URI, "Poses");
    RegisterPosesDataCallback(m_callback);
  }

  void Clear() {
    m_Poses = nullptr;
  }

  void RegisterPosesDataCallback(PosesDriverDataCallback callback) {
    m_callback = callback;
    if( m_Poses ){
      m_Poses->RegisterPosesDataCallback( callback );
    }else{
      std::cerr << "ERROR: no driver initialized!\n";
    }
    return;
  }

  void RegisterPosesFinishedLoadingDataCallback(PosesDriverFinishedLoadingDataCallback callback) {
    m_finishedCallback = callback;
    if( m_Poses ){
      m_Poses->RegisterPosesFinishedLoadingDataCallback( callback );
    }else{
      std::cerr << "ERROR: no driver initialized!\n";
    }
    return;
  }


  std::string GetDeviceProperty(const std::string& sProperty) {
    return m_Poses->GetDeviceProperty(sProperty);
  }

  bool IsRunning() const override {
    return m_Poses && m_Poses->IsRunning();
  }


 protected:
  hal::Uri                                          m_URI;
  std::shared_ptr<PosesDriverInterface>     m_Poses;
  PosesDriverDataCallback                   m_callback;
  PosesDriverFinishedLoadingDataCallback    m_finishedCallback;

};
} /* namespace hal */
