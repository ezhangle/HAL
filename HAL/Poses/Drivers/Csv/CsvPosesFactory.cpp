#include <HAL/Devices/DeviceFactory.h>
#include <HAL/Utils/StringUtils.h>
#include <HAL/Poses/PosesDriverInterface.h>


#include "CsvPosesDriver.h"

namespace hal
{

class CsvPosesFactory : public DeviceFactory<PosesDriverInterface>
{
public:
    CsvPosesFactory(const std::string& name)
        : DeviceFactory<PosesDriverInterface>(name)
    {
        Params() = {
        };
    }

    std::shared_ptr<PosesDriverInterface> GetDevice(const Uri& uri)
    {
        const std::string sDataSourceDir = hal::ExpandTildePath(uri.url);
        const std::string sFilePoses = uri.properties.Get( "Poses", sDataSourceDir+"/poses.txt");
        const std::string sFileTimestamp  = uri.properties.Get( "Timestamp", sDataSourceDir+"/timestamp.txt");
        
        CsvPosesDriver* pDriver = new CsvPosesDriver(sFilePoses, sFileTimestamp);
        return std::shared_ptr<PosesDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static CsvPosesFactory g_CsvPosesFactory("csv");

}
