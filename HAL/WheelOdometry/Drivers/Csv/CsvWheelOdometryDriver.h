#pragma once

#include <thread>
#include <fstream>

#include <HAL/WheelOdometry/WheelOdometryDriverInterface.h>
#include <HAL/Messages/Pose.h>


namespace hal {

class CsvWheelOdometryDriver : public WheelOdometryDriverInterface
{
public:
    CsvWheelOdometryDriver(const std::string sFileWheelSpeed, const std::string sFileSteeringAngle,
               const std::string sFileTimestamp
               );
    ~CsvWheelOdometryDriver();

    void RegisterWheelOdometryDataCallback(WheelOdometryDriverDataCallback callback);
    void RegisterWheelOdometryFinishedLoadingDataCallback(WheelOdometryDriverFinishedLoadingDataCallback callback);


    Sophus::SE3Group<double> IntegrateWheelSpeed( const double start_time,
                                      const std::vector<WheelOdometryMsg>& measurements,
                                      const double end_time);

    std::vector<hal::PoseMsg> IntegrateWheelSpeed(
                              const std::vector<WheelOdometryMsg>& measurements);

    bool IsRunning() const override {
      return m_bShouldRun;
    }

private:
    void _ThreadCaptureFunc();
    bool _GetNextTime( double& dNextTime, double& dNextTimePPS );

    bool                              m_bHaveWheelSpeed;
    bool                              m_bHaveSteerAngle;
    std::string                       m_sDataSourceDir;
    std::ifstream                     m_pFileTime;
    std::ifstream                     m_pFileWheelSpeed;
    std::ifstream                     m_pFileSteerAngle;
    volatile bool                     m_bShouldRun;
    double                            m_dNextTime;
    double                            m_dNextTimePPS;
    std::thread                       m_DeviceThread;
    WheelOdometryDriverDataCallback   m_WheelOdometryCallback;
    WheelOdometryDriverFinishedLoadingDataCallback   m_WheelOdometryFinishedCallback;


};

} /* namespace */
