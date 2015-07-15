#include <iostream>

#include <HAL/Utils/TicToc.h>
#include <HAL/Utils/StringUtils.h>
#include <HAL/Devices/DeviceException.h>
#include <HAL/Devices/DeviceTime.h>
#include <stdexcept>

#include "CsvWheelOdometryDriver.h"

namespace hal
{

///////////////////////////////////////////////////////////////////////////////
CsvWheelOdometryDriver::CsvWheelOdometryDriver(const std::string sFileWheelSpeed,
        const std::string sFileSteeringAngle,
        const std::string sFileTimestamp
        )
{


    // open WheelOdometry log files
    if( sFileTimestamp.empty() ) {
        throw hal::DeviceException("WheelOdometryLog: File with timestamps is required.");
    } else {
        m_pFileTime.open( sFileTimestamp.c_str() );
        if( m_pFileTime.is_open() == false ) {
            throw hal::DeviceException("WheelOdometryLog: Couldn't open required timestamp file '"+sFileTimestamp+"'");
        }
    }

    if( sFileWheelSpeed.empty() == false ) {
        m_pFileWheelSpeed.open( sFileWheelSpeed.c_str() );
        if( m_pFileWheelSpeed.is_open() == false ) {
            std::cerr << "WheelOdometryLog: Couldn't open wheel speed file '" << sFileWheelSpeed << "'" << std::endl;
        } else {
            m_bHaveWheelSpeed = true;
        }
    }

    if( sFileSteeringAngle.empty() == false ) {
        m_pFileSteerAngle.open( sFileSteeringAngle.c_str() );
        if( m_pFileSteerAngle.is_open() == false ) {
            std::cerr << "WheelOdometryLog: Couldn't open steer angle file '" << sFileSteeringAngle << "'" << std::endl;
        } else {
            m_bHaveSteerAngle = true;
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
CsvWheelOdometryDriver::~CsvWheelOdometryDriver()
{
    // close capture thread
    m_bShouldRun = false;

    // wait for capture thread to die
    m_DeviceThread.join();

    // close IMU log files
    if( m_pFileTime.is_open() ) {
        m_pFileTime.close();
    }
    if( m_pFileWheelSpeed.is_open() ) {
        m_pFileWheelSpeed.close();
    }
    if( m_pFileSteerAngle.is_open() ) {
        m_pFileSteerAngle.close();
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void CsvWheelOdometryDriver::RegisterWheelOdometryDataCallback(WheelOdometryDriverDataCallback callback)
{
    m_WheelOdometryCallback = callback;
    if( !m_DeviceThread.joinable() ) {
        // start capture thread
        m_bShouldRun = true;
        m_DeviceThread = std::thread( &CsvWheelOdometryDriver::_ThreadCaptureFunc, this );
    }
}

///////////////////////////////////////////////////////////////////////////////
void CsvWheelOdometryDriver::_ThreadCaptureFunc()
{
    while( m_bShouldRun ) {
        hal::DeviceTime::WaitForTime( m_dNextTime );

        //---------------------------------------------------------

        // get data and pack
        // we already have the timestamp so no need to read it!
        std::string     sValue;

        hal::WheelOdometryMsg dataWheelOdometry;

        if( m_bHaveWheelSpeed ) {
            // Assuming wheel speed columns are in order: fr, fl, rr, rl

            getline ( m_pFileWheelSpeed, sValue, ',' );
            dataWheelOdometry.set_front_right( atof( sValue.c_str() ) );

            getline ( m_pFileWheelSpeed, sValue, ',' );
            dataWheelOdometry.set_front_left( atof( sValue.c_str() ) );

            getline ( m_pFileWheelSpeed, sValue, ',' );
            dataWheelOdometry.set_rear_right( atof( sValue.c_str() ) );

            getline ( m_pFileWheelSpeed, sValue, ',' );
            dataWheelOdometry.set_rear_left( atof( sValue.c_str() ) );

        }

        if( m_bHaveSteerAngle ) {
            // Assuming one steerangle per line, if there are more items in a line
            // use only the first row.

            getline ( m_pFileSteerAngle, sValue);
            std::vector<std::string> items = hal::Split(sValue, ',');

            dataWheelOdometry.set_steer_angle( atof( items[0].c_str() ) );
        }


        if( (m_bHaveWheelSpeed || m_bHaveSteerAngle) && m_WheelOdometryCallback ) {
            dataWheelOdometry.set_device_time(m_dNextTimePPS);
            dataWheelOdometry.set_system_time(m_dNextTime);
            m_WheelOdometryCallback( dataWheelOdometry );
        }

        //---------------------------------------------------------

        // break if EOF
        if( _GetNextTime( m_dNextTime, m_dNextTimePPS ) == false ) {
            break;
        }

        // pop front and push next timestamp to queue
        hal::DeviceTime::PopAndPushTime( m_dNextTime );
    }
    m_bShouldRun = false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline bool CsvWheelOdometryDriver::_GetNextTime(
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
