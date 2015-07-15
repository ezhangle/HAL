#include <HAL/Devices/DeviceFactory.h>
#include <HAL/Utils/StringUtils.h>
#include <HAL/WheelOdometry/WheelOdometryDriverInterface.h>


#include "CsvWheelOdometryDriver.h"

namespace hal
{

class CsvFactory : public DeviceFactory<WheelOdometryDriverInterface>
{
public:
    CsvFactory(const std::string& name)
        : DeviceFactory<WheelOdometryDriverInterface>(name)
    {
        Params() = {
        };
    }

    std::shared_ptr<WheelOdometryDriverInterface> GetDevice(const Uri& uri)
    {
        const std::string sDataSourceDir = hal::ExpandTildePath(uri.url);
        const std::string sFileWheelSpeed = uri.properties.Get( "WheelSpeed", sDataSourceDir+"/wheelspeed.txt");
        const std::string sFileAngle  = uri.properties.Get( "Angle", sDataSourceDir+"/steerangle.txt");
        const std::string sFileTimestamp  = uri.properties.Get( "Timestamp", sDataSourceDir+"/timestamp.txt");
        
        CsvWheelOdometryDriver* pDriver = new CsvWheelOdometryDriver(sFileWheelSpeed, sFileAngle, sFileTimestamp);
        return std::shared_ptr<WheelOdometryDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static CsvFactory g_CsvFactory("csv");

}
