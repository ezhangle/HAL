#pragma once

#include <thread>
#include <fstream>
#include <HAL/Messages/Pose.h>
#include <HAL/Poses/PosesDriverInterface.h>


namespace hal {

class CsvPosesDriver : public PosesDriverInterface
{
public:
    CsvPosesDriver(const std::string sFilePoses,
               const std::string sFileTimestamp);
    ~CsvPosesDriver();

    void RegisterPosesDataCallback(PosesDriverDataCallback callback);
    void RegisterPosesFinishedLoadingDataCallback(PosesDriverFinishedLoadingDataCallback callback);

    bool IsRunning() const override {
      return m_bShouldRun;
    }

private:
    void _ThreadCaptureFunc();
    bool _GetNextTime( double& dNextTime, double& dNextTimePPS );

    bool                              m_bHavePoses;
    std::string                       m_sDataSourceDir;
    std::ifstream                     m_pFileTime;
    std::ifstream                     m_pFilePoses;
    volatile bool                     m_bShouldRun;
    double                            m_dNextTime;
    double                            m_dNextTimePPS;
    std::thread                       m_DeviceThread;
    PosesDriverDataCallback           m_PosesCallback;
    PosesDriverFinishedLoadingDataCallback   m_PosesFinishedLoadingDataCallback;

};

} /* namespace */
