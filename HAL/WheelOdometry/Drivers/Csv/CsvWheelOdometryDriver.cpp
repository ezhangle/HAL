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
CsvWheelOdometryDriver::CsvWheelOdometryDriver(
        const std::string sFileWheelSpeed,
        const std::string sFileSteeringAngle,
        const std::string sFileTimestamp
        )
{

    m_bShouldRun = false;

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

    // close Odometry log files
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


/////////////////////////////////////////////////////////////////////////////////////////
void CsvWheelOdometryDriver::RegisterWheelOdometryFinishedLoadingDataCallback(WheelOdometryDriverFinishedLoadingDataCallback callback)
{
    m_WheelOdometryFinishedCallback = callback;
}

///////////////////////////////////////////////////////////////////////////////
void CsvWheelOdometryDriver::_ThreadCaptureFunc()
{
    while( m_bShouldRun ) {
        hal::DeviceTime::WaitForTime( m_dNextTime );
        //---------------------------------------------------------
        double km_h_to_m_s = 1/3.6;
        // get data and pack
        // we already have the timestamp so no need to read it!
        std::string     sValue;

        hal::WheelOdometryMsg dataWheelOdometry;

        if( m_bHaveWheelSpeed ) {
            // Assuming wheel speed columns are in order: rr, rl, fr, fl

            getline ( m_pFileWheelSpeed, sValue, ',' );
            dataWheelOdometry.set_rear_right( atof( sValue.c_str() )*km_h_to_m_s );

            getline ( m_pFileWheelSpeed, sValue, ',' );
            dataWheelOdometry.set_rear_left( atof( sValue.c_str() )*km_h_to_m_s );

            getline ( m_pFileWheelSpeed, sValue, ',' );
            dataWheelOdometry.set_front_right( atof( sValue.c_str() )*km_h_to_m_s );

            getline ( m_pFileWheelSpeed, sValue, ',' );
            dataWheelOdometry.set_front_left( atof( sValue.c_str() )*km_h_to_m_s );

        }

        if( m_bHaveSteerAngle ) {

            // Assuming one steerangle per line, if there are more items in a line
            // use only the first row.

            getline ( m_pFileSteerAngle, sValue);
            std::vector<std::string> items = hal::Split(sValue, ',');
            // In order to use TOYOTA steering angle, convert to radians and divide by 17.4

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
          if(m_WheelOdometryFinishedCallback){
            m_WheelOdometryFinishedCallback();
          }
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

// TODO: This should be moved to a different, common file for all the 'Wheel Odometry' drivers to use
////////////////////////////////////////////////////////////////////////////////
// BEGIN DIFFERENTIAL DRIVE MODEL
////////////////////////////////////////////////////////////////////////////////

const double kTrackWidth = 1.61;
//const double kWheelBase  = 3.09;

WheelOdometryMsg Interpolate(const WheelOdometryMsg& wheel_speed1,
                                  const WheelOdometryMsg& wheel_speed2,
                                  const double timestamp) {
  assert(wheel_speed1.device_time() <= timestamp &&
         timestamp <= wheel_speed2.device_time());

  double ratio = (timestamp - wheel_speed1.device_time())
      / (wheel_speed2.device_time() - wheel_speed1.device_time());

  WheelOdometryMsg wheel_speed;
  wheel_speed.set_speed( wheel_speed1.speed()       +
      (wheel_speed2.speed()       - wheel_speed1.speed())       * ratio );
  wheel_speed.set_front_right( wheel_speed1.front_right() +
      (wheel_speed2.front_right() - wheel_speed1.front_right()) * ratio );
  wheel_speed.set_front_left( wheel_speed1.front_left()  +
      (wheel_speed2.front_left()  - wheel_speed1.front_left())  * ratio );
  wheel_speed.set_rear_right( wheel_speed1.rear_right()  +
      (wheel_speed2.rear_right()  - wheel_speed1.rear_right())  * ratio );
  wheel_speed.set_rear_left( wheel_speed1.rear_left()   +
      (wheel_speed2.rear_left()   - wheel_speed1.rear_left())   * ratio );
  wheel_speed.set_device_time( timestamp );

  // Note: The steering angle is not used in this model, so it's not
  // currently being interpolated.

  return wheel_speed;
}

////////////////////////////////////////////////////////////////////////////////
void IntegrateLocalWheelSpeed(const WheelOdometryMsg& wheel_speed,
                         double* incremental_x,
                         double* incremental_y,
                         double* incremental_yaw,
                         double* incremental_speed,
                         double* incremental_timestamp) {
  if (*incremental_timestamp < wheel_speed.device_time()) {
    /* Can message seems to be reversed (front <-> rear). */
    double rr = wheel_speed.rear_right();
    double rl = wheel_speed.rear_left();
    double dt = wheel_speed.device_time() - *incremental_timestamp;

    double speed = (rr + rl) * 0.5;

    /* Computes new Differential Drive parameters. */
    if (fabs(rr) > 0.001 || fabs(rl) > 0.001) {
      if (fabs(rr - rl) < 0.001) {
        /* straight line projected along path */
        *incremental_x += cos(*incremental_yaw) * speed * dt;
        *incremental_y += sin(*incremental_yaw) * speed * dt;
      } else {
        double w = (rr - rl) / kTrackWidth;
        double R = kTrackWidth * 0.5 * (rr + rl) / (rr - rl);
        double icc_x = *incremental_x - R * sin(*incremental_yaw); double icc_y = *incremental_y + R * cos(*incremental_yaw);

        double wdt = w * dt;
        double new_x = cos(wdt) * (*incremental_x - icc_x) - sin(wdt) * (*incremental_y - icc_y) + icc_x;
        double new_y = sin(wdt) * (*incremental_x - icc_x) + cos(wdt) * (*incremental_y - icc_y) + icc_y;
        double new_theta = *incremental_yaw + wdt;

        *incremental_x = new_x;
        *incremental_y = new_y;
        *incremental_yaw = new_theta;
      }
    }

    *incremental_yaw = fmod(*incremental_yaw + 2*M_PI, 2*M_PI);

    *incremental_speed = speed;
    std::cerr << "dt: " << dt << std::endl;
    /* Updates timestamp. */
    *incremental_timestamp = wheel_speed.device_time();
  }
}

////////////////////////////////////////////////////////////////////////////////
hal::PoseMsg ConvertPose(const double x, const double y, const double z,
                    const double yaw, const double speed,
                    const double timestamp) {
  Sophus::SE3Group<double> pose;

  pose.translation() = Eigen::Vector3d(x, y, z);

  Eigen::Matrix3d rotation_matrix;
  rotation_matrix =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
  pose.setRotationMatrix(rotation_matrix);

//  //pose.speed = speed;
  std::cerr << "new pose: " << std::endl <<
               "translation:" << pose.translation().transpose() << std::endl <<
               "rotation: " << pose.rotationMatrix().eulerAngles(0, 1, 2).transpose() << std::endl << std::endl;

  hal::PoseMsg halPose;
  WritePoseSE3(pose, &halPose);

  halPose.set_device_time(timestamp);

  return halPose;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<hal::PoseMsg> CsvWheelOdometryDriver::IntegrateWheelSpeed(
                                                const std::vector<WheelOdometryMsg>& measurements){

  std::vector<hal::PoseMsg> poses;

  if(measurements.size() > 2){
    double pose_x = 0;
    double pose_y = 0;
    double pose_z = 0;
    double pose_yaw = 0;
    double pose_speed = 0;

    hal::PoseMsg initial_pose = hal::PoseMsg();
    /* Sets up initial pose. Starts at the origin with no rotation*/
    poses.push_back(initial_pose);
    double previous_timestamp = measurements[0].device_time();
    for (const auto& current_measurement : measurements) {
      if(current_measurement.device_time() <= previous_timestamp){
        continue;
      }

      //std::cerr << "Integrating wheel speed from ts: "<< previous_timestamp << "to: " << current_measurement.device_time() <<
      //             std::endl;

      IntegrateLocalWheelSpeed(current_measurement, &pose_x, &pose_y, &pose_yaw, &pose_speed,
                          &previous_timestamp);
      poses.push_back(ConvertPose(pose_x, pose_y, pose_z, pose_yaw, pose_speed,
                                  previous_timestamp));

      previous_timestamp = current_measurement.device_time();
    }
  }

  return poses;

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Sophus::SE3Group<double> CsvWheelOdometryDriver::IntegrateWheelSpeed(
                                                const double start_time,
                                                const std::vector<WheelOdometryMsg>& measurements,
                                                const double end_time){

  Sophus::SE3Group<double> t_12;

//  double pose_x;
//  double pose_y;
//  double pose_z = 0;
//  double pose_yaw;
//  double pose_speed;

//  /* Sets up initial pose. */
//  //poses.push_back(initial_pose);

//  double previous_timestamp = measurements[0].device_time();
//  uint i = 0;
//  for (const auto& current_measurement : measurements) {
//    if (start_time < current_measurement.device_time()) {

//      // Found the first wheel speed measurement after start_time, interpolate to get a better
//      // approximation
//      std::shared_ptr<WheelOdometryMsg> start_measurement = i > 0 ? Interpolate( measurements, current_measurement, start_time ) :
//                                  current_measurement;

//      IntegrateWheelSpeed(start_measurement, &pose_x, &pose_y, &pose_yaw, &pose_speed,
//                          &previous_timestamp);
//      poses.push_back(ConvertPose(pose_x, pose_y, pose_z, pose_yaw, pose_speed,
//                                  previous_timestamp));
//      if (integration_time != 0.0 &&
//          integration_time < current_measurement.timestamp - initial_pose.timestamp) {
//        break;
//      }
//    }
//    i++;
//    previous_timestamp = current_measurement.device_time();
//  }

  return t_12;

}

///////////////////////////////////////////////////////////////////////////////////////////////////
// END DIFFERENTIAL DRIVE MODEL
//////////////////////////////////////////////////////////////////////////////////////////////////


}


