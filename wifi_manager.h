#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "ns3/network-module.h"
#include "ns3/wifi-module.h"

#include <vector>

class WifiManager
{
public:

    WifiManager();

    void Install(const std::vector<ns3::Ptr<ns3::Node>>& nodes);

private:

    ns3::NetDeviceContainer m_devices;
};

#endif
