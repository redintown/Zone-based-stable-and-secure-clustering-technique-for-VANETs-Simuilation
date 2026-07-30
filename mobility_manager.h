// File: mobility_manager.h
#ifndef MOBILITY_MANAGER_H
#define MOBILITY_MANAGER_H

#include "vehicle.h"

#include "ns3/mobility-module.h"
#include "ns3/network-module.h"

#include <map>
#include <string>
#include <vector>

class MobilityManager
{
public:

    MobilityManager();

    void InitializeNode(const std::string& id,
                        ns3::Ptr<ns3::Node> node);

    void UpdatePosition(const ns3::Vehicle& vehicle);

private:

    std::map<std::string,
             ns3::Ptr<ns3::ConstantPositionMobilityModel>> m_mobility;
};

#endif
