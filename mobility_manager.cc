// File: mobility_manager.cc
#include "mobility_manager.h"

#include <iostream>
#include <string>

using namespace ns3;

MobilityManager::MobilityManager()
{
}

void
MobilityManager::InitializeNode(const std::string& id,
                                Ptr<Node> node)
{
    Ptr<ConstantPositionMobilityModel> mobility =
        CreateObject<ConstantPositionMobilityModel>();

    node->AggregateObject(mobility);

    m_mobility[id] = mobility;
}

void
MobilityManager::UpdatePosition(const ns3::Vehicle& vehicle)
{
    std::string vid = std::to_string(vehicle.GetVehicleId());
    auto it = m_mobility.find(vid);

    if (it == m_mobility.end())
    {
        return;
    }

    Vector position(vehicle.GetCurrentPosition().x,
                    vehicle.GetCurrentPosition().y,
                    0.0);

    it->second->SetPosition(position);

    std::cout
        << "[Mobility] "
        << vid
        << " -> ("
        << vehicle.GetCurrentPosition().x
        << ", "
        << vehicle.GetCurrentPosition().y
        << ")"
        << std::endl;
}
