/*

    Drivers MUST register themselves by instantiating a global
    CameraDriverRegisteryEntry singleton, typically at the bottom of the header
    in which the driver class is declared.

    E.g., at the end of the FileReaderDriver.h, you'll note the following line:

    static CameraDriverRegisteryEntry<FileReaderDriver> _Reg( "FileReader" );

    When this variable is instantiated (which happens before main runs) the
    CameraDriverRegisteryEntry constructor registers the "FileReader" name
    and a function pointer to a function that a CameraDevice can then use to
    instantiate a FileReader driver. 

    Thus, given a CameraDevice C, when we call C.InitDriver("FileReader") it
    will call the appropriate function pointer to set it's internal driver to
    be a FileReaderDriver.

 */

#ifndef _CAMERA_DRIVER_REGISTERY_H_
#define _CAMERA_DRIVER_REGISTERY_H_


CameraDriver* CreateCameraDriver( const std::string& sDriverName );

extern std::map<std::string,CameraDriver*(*)()> g_mCameraDriverTable;

class CameraDriver;

template <class T>
class CameraDriverRegisteryEntry
{
    public:
        //////////////////////////////////////////////////////////////////////
        // Generic creation function
        static T* DriverCreationFunction()
        {
            return new T;
        }

        //////////////////////////////////////////////////////////////////////
        // Register a driver
        CameraDriverRegisteryEntry(
                const std::string& sDriverName
                )
        {
//            std::cout << "Registered '" << sDriverName << "' driver.\n";
//            printf("g_mDriverTable lives at %p\n", &g_mDriverTable );

            g_mCameraDriverTable[sDriverName]
                = (CameraDriver* (*)())DriverCreationFunction;
//            std::cout << "Known:\n";
//            std::map<std::string,CameraDriver*(*)()>::iterator it;
//            for( it = g_mDriverTable.begin(); it != g_mDriverTable.end(); it++ ){
//                std::cerr << "\t" << it->first << std::endl;
//            }
        }
};

#endif
