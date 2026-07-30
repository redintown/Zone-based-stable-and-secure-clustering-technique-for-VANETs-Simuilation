#include "hello_manager.h"

#include <iostream>
#include <string>

using namespace ns3;

HelloManager::HelloManager()
{
}

void
HelloManager::Broadcast(std::vector<ns3::Vehicle>& vehicles)
{
    m_helloTable.clear();

    for (auto& vehicle : vehicles)
    {
        HelloInfo info;

        info.vehicleId = std::to_string(vehicle.GetVehicleId());
        info.laneId = vehicle.GetLaneId();
        info.roadId = std::to_string(vehicle.GetRoadId());

        info.x = vehicle.GetCurrentPosition().x;
        info.y = vehicle.GetCurrentPosition().y;

        info.speed = vehicle.GetSpeed();
        info.angle = vehicle.GetDirectionAngle();

        info.timestamp = 0.0;

        m_helloTable[std::to_string(vehicle.GetVehicleId())] = info;
    }

    std::cout
        << "[HELLO] "
        << m_helloTable.size()
        << " HELLO messages generated."
        << std::endl;
}

const std::map<std::string, HelloInfo>&
HelloManager::GetHelloTable() const
{
    return m_helloTable;
}
