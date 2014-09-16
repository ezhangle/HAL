#include <HAL/Devices/DeviceFactory.h>
#include "Freenect2Driver.h"

namespace hal
{

class Freenect2Factory : public DeviceFactory<CameraDriverInterface>
{
public:
    Freenect2Factory(const std::string& name)
        : DeviceFactory<CameraDriverInterface>(name)
    {
        Params() = {
            {"size", "640x480", "Capture resolution."},
            {"rgb", "true", "Capture RGB image."},
            {"depth", "true", "Capture depth image."},
            {"ir", "false", "Capture infrared image."},
            {"color", "true", "Keep color in RGB image"},
        };
    }

    std::shared_ptr<CameraDriverInterface> GetDevice(const Uri& uri)
    {
        ImageDim Dims       = uri.properties.Get("size", ImageDim(1024,576));
        bool bRGB           = uri.properties.Get("rgb", true);
        bool bDepth         = uri.properties.Get("depth", true);
        bool bIR            = uri.properties.Get("ir", false);
        bool bColor         = uri.properties.Get("color", true);

        Freenect2Driver* pDriver = new Freenect2Driver(
                    Dims.x, Dims.y, bRGB, bDepth, bIR, bColor );
        return std::shared_ptr<CameraDriverInterface>( pDriver );
    }
};

// Register this factory by creating static instance of factory
static Freenect2Factory g_Freenect2Factory("freenect2");

}
