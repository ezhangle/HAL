#include <iostream>

#include <HAL/Utils/TicToc.h>
#include <HAL/Utils/StringUtils.h>
#include <HAL/Devices/DeviceException.h>
#include <HAL/Devices/DeviceTime.h>
#include <stdexcept>

#include "CsvPosesDriver.h"

namespace hal
{

///////////////////////////////////////////////////////////////////////////////
CsvPosesDriver::CsvPosesDriver(
        const std::string sFilePoses,
        const std::string sFileTimestamp
        )
{

    m_bShouldRun = false;

    // open Poses log files
    if( sFileTimestamp.empty() ) {
        throw hal::DeviceException("PosesLog: File with timestamps is required.");
    } else {
        m_pFileTime.open( sFileTimestamp.c_str() );
        if( m_pFileTime.is_open() == false ) {
            throw hal::DeviceException("PosesLog: Couldn't open required timestamp file '"+sFileTimestamp+"'");
        }
    }

    if( sFilePoses.empty() == false ) {
        m_pFilePoses.open( sFilePoses.c_str() );
        if( m_pFilePoses.is_open() == false ) {
            std::cerr << "PosesLog: Couldn't open poses file '" << sFilePoses << "'" << std::endl;
        } else {
            m_bHavePoses = true;
        }
    }


    // read one timestamp.. return false if error occurs
    if( _GetNextTime( m_dNextTime, m_dNextTimePPS ) == false ) {
        throw std::runtime_error("Umm..");
    }

    // push timestamp to VD queue
    hal::DeviceTime::PushTime( m_dNextTime );

}

///////////////////////////////////////////////////////////////////////////////
CsvPosesDriver::~CsvPosesDriver()
{
    // close capture thread
    m_bShouldRun = false;

    // wait for capture thread to die
    m_DeviceThread.join();

    // close Odometry log files
    if( m_pFileTime.is_open() ) {
        m_pFileTime.close();
    }
    if( m_pFilePoses.is_open() ) {
        m_pFilePoses.close();
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void CsvPosesDriver::RegisterPosesDataCallback(PosesDriverDataCallback callback)
{
    m_PosesCallback = callback;
    if( !m_DeviceThread.joinable() ) {
        // start capture thread
        m_bShouldRun = true;
        m_DeviceThread = std::thread( &CsvPosesDriver::_ThreadCaptureFunc, this );
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void CsvPosesDriver::RegisterPosesFinishedLoadingDataCallback(PosesDriverFinishedLoadingDataCallback callback)
{
    m_PosesFinishedLoadingDataCallback = callback;
}

///////////////////////////////////////////////////////////////////////////////
void CsvPosesDriver::_ThreadCaptureFunc()
{
    while( m_bShouldRun ) {
        hal::DeviceTime::WaitForTime( m_dNextTime );

        //---------------------------------------------------------

        // get data and pack
        // we already have the timestamp so no need to read it!
        std::string     sValue;

        Sophus::SE3Group<double> pose;
        hal::PoseMsg dataPose;

        double x;
        double y;
        double z;
        double p;
        double q;
        double r;

        if( m_bHavePoses ) {
            // Assuming pose columns are in order: x, y, z, roll, pitch, yaw

            getline ( m_pFilePoses, sValue, ',' );
            x = atof( sValue.c_str() );

            getline ( m_pFilePoses, sValue, ',' );
            y = atof( sValue.c_str() );

            getline ( m_pFilePoses, sValue, ',' );
            z = atof( sValue.c_str() );

            getline ( m_pFilePoses, sValue, ',' );
            p = atof( sValue.c_str() );

            getline ( m_pFilePoses, sValue, ',' );
            q = atof( sValue.c_str() );

            getline ( m_pFilePoses, sValue, ',' );
            r = atof( sValue.c_str() );

            z = 0; // temporarily fix z to zero since we are assuming planar motion
            pose.translation() = Eigen::Vector3d(x, y, z);
            Eigen::Matrix3d rotation_matrix;
            rotation_matrix =
                Eigen::AngleAxisd(r, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) * // temporarily fix roll and pitch to 0 since we are assuming
                Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());  // planar motion
            pose.setRotationMatrix(rotation_matrix);

//            std::cerr << "Setting rotation matrix: " << std::endl <<
//                         rotation_matrix << std::endl <<
//                         "and translation: " << std::endl
//                      << pose.translation() << std::endl;


            WritePoseSE3(pose, &dataPose);
        }



        if( m_bHavePoses && m_PosesCallback ) {
            dataPose.set_device_time(m_dNextTimePPS);
            dataPose.set_system_time(m_dNextTime);
            m_PosesCallback( dataPose );
        }

        //---------------------------------------------------------

        // break if EOF
        if( _GetNextTime( m_dNextTime, m_dNextTimePPS ) == false ) {
            if(m_PosesFinishedLoadingDataCallback){
              m_PosesFinishedLoadingDataCallback();
            }
            break;
        }

        // pop front and push next timestamp to queue
        hal::DeviceTime::PopAndPushTime( m_dNextTime );
    }
    m_bShouldRun = false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline bool CsvPosesDriver::_GetNextTime(
        double& dNextTime,                  //< Output
        double& dNextTimePPS                //< Output
        )
{
    std::string sValue;

    getline ( m_pFileTime, sValue, ',' );
    dNextTime = atof( sValue.c_str() );
    getline ( m_pFileTime, sValue );
    dNextTimePPS = atof( sValue.c_str() );

    if( m_pFileTime.eof() ) {
        return false;
    }

    return true;
}

}


