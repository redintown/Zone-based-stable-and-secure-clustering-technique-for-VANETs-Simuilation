#include "wifi_manager.h"

#include <iostream>

using namespace ns3;

WifiManager::WifiManager()
{
}

void
WifiManager::Install(const std::vector<Ptr<Node>>& nodes)
{
    NodeContainer nodeContainer;

    for (auto node : nodes)
    {
        nodeContainer.Add(node);
    }

    WifiHelper wifi;

    wifi.SetStandard(WIFI_STANDARD_80211g);

    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");

    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();

    // ns-3.46 compatible
    YansWifiPhyHelper phy;

    phy.SetChannel(channel.Create());

    m_devices = wifi.Install(
        phy,
        mac,
        nodeContainer);

    std::cout
        << "[WiFi] Installed on "
        << nodeContainer.GetN()
        << " nodes."
        << std::endl;
}
