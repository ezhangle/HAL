// hack to enable sleep_for (GCC < 4.8)
#ifndef _GLIBCXX_USE_NANOSLEEP
#define _GLIBCXX_USE_NANOSLEEP
#endif  // _GLIBCXX_USE_NANOSLEEP

#include "NodeIMUDriver.h"

using namespace hal;


/////////////////////////////////////////////////////////////////////////////////////////
NodeIMUDriver::NodeIMUDriver(const std::string& sHost)
    : m_host(sHost), m_running(false), m_callback(nullptr)
{
    if( m_node.Subscribe("IMU", sHost) == false ) {
        std::cerr << "HAL: Error subscribing to remote node." << std::endl;
    }
}


/////////////////////////////////////////////////////////////////////////////////////////
void NodeIMUDriver::_ThreadFunc()
{
    pb::ImuMsg pbMsg;
    while( m_running ) {
        pbMsg.Clear();
        if( m_node.ReadBlocking("IMU", pbMsg) == false ) {
            std::cerr << "HAL: Error reading node publisher." << std::endl;
            continue;
        }
        m_callback(pbMsg);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
NodeIMUDriver::~NodeIMUDriver()
{
    m_running = false;
    if( m_callbackThread.joinable() ) {
        m_callbackThread.join();
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void NodeIMUDriver::RegisterIMUDataCallback(IMUDriverDataCallback callback)
{
    m_callback = callback;
    m_running = true;
    m_callbackThread = std::thread( &NodeIMUDriver::_ThreadFunc, this );
}
