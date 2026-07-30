#include "zone_manager.h"
#include <ns3/log.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("ZoneManager");

ZoneManager::ZoneManager ()
    : m_rsuId (0),
      m_rsuPosition (0.0, 0.0, 0.0),
      m_rsuCoverage (900.0),
      m_numZonesPerLane (3)
{
    // Default Y-coordinates for the three lanes
    m_laneYCoords[1] = 10.0;
    m_laneYCoords[2] = 20.0;
    m_laneYCoords[3] = 30.0;
}

double ZoneManager::GetLaneAverageSpeed (uint32_t laneId)
{
    switch (laneId) {
        case 1:
            return 15.0; // Lane_1: 10-20 m/s -> Average = 15.0 m/s
        case 2:
            return 25.5; // Lane_2: 21-30 m/s -> Average = 25.5 m/s
        case 3:
            return 35.5; // Lane_3: 31-40 m/s -> Average = 35.5 m/s
        default:
            return 15.0;
    }
}

double ZoneManager::CalculateDeadZoneWidth (double laneAvgSpeed, double tHello)
{
    // Eq. (1): Dead Zone = (S_L,avg * t_HELLO) / 4
    return (laneAvgSpeed * tHello) / 4.0;
}

double ZoneManager::CalculateDistanceToRsu (const Vector &rsuPos, const Vector &vehiclePos)
{
    // Eq. (4): d_{r,n} = sqrt((x_r - x_n)^2 + (y_r - y_n)^2)
    double dx = rsuPos.x - vehiclePos.x;
    double dy = rsuPos.y - vehiclePos.y;
    return std::sqrt (dx * dx + dy * dy);
}

double ZoneManager::CalculateDistanceToCenter (const Vector &centerPos, const Vector &vehiclePos)
{
    // Euclidean distance to fixed zone center point
    double dx = centerPos.x - vehiclePos.x;
    double dy = centerPos.y - vehiclePos.y;
    return std::sqrt (dx * dx + dy * dy);
}

void ZoneManager::SetLaneYCoordinate (uint32_t laneId, double y)
{
    m_laneYCoords[laneId] = y;
}

double ZoneManager::GetLaneYCoordinate (uint32_t laneId) const
{
    auto it = m_laneYCoords.find (laneId);
    if (it != m_laneYCoords.end ()) {
        return it->second;
    }
    return 10.0 * static_cast<double> (laneId);
}

void ZoneManager::InitializeRsuZones (uint32_t rsuId, const Vector &rsuPos, double rsuCoverage, uint32_t numZones)
{
    m_rsuId = rsuId;
    m_rsuPosition = rsuPos;
    m_rsuCoverage = rsuCoverage;
    m_numZonesPerLane = numZones;

    m_laneZones.clear ();

    double coverageStartX = rsuPos.x - (rsuCoverage / 2.0);
    double zoneWidth = rsuCoverage / static_cast<double> (numZones);

    for (uint32_t laneId = 1; laneId <= 3; ++laneId) {
        double avgSpeed = GetLaneAverageSpeed (laneId);
        double deadZoneW = CalculateDeadZoneWidth (avgSpeed, 1.0);
        double laneY = GetLaneYCoordinate (laneId);

        std::vector<Zone> zones;
        for (uint32_t z = 1; z <= numZones; ++z) {
            Zone zone;
            zone.zoneId = z;
            zone.laneId = laneId;
            zone.rsuId = rsuId;
            zone.startX = coverageStartX + (z - 1) * zoneWidth;
            zone.endX = zone.startX + zoneWidth;
            zone.laneY = laneY;
            zone.centerPoint = Vector ((zone.startX + zone.endX) / 2.0, laneY, 0.0);
            
            zone.deadZoneWidth = deadZoneW;
            zone.activeZoneStartX = zone.startX + deadZoneW;
            zone.activeZoneEndX = zone.endX - deadZoneW;
            zone.activeZoneLength = zone.activeZoneEndX - zone.activeZoneStartX;

            std::cout
                << "[ZONE INIT] "
                << "Lane=" << laneId
                << " Zone=" << zone.zoneId
                << " Start=" << zone.startX
                << " End=" << zone.endX
                << " Center=("
                << zone.centerPoint.x << ", "
                << zone.centerPoint.y << ")"
                << std::endl;
        
            zones.push_back (zone);
        }
        m_laneZones[laneId] = zones;
    }
}

uint32_t ZoneManager::FindNearestZoneId (const Vector &vehiclePos, uint32_t laneId) const
{
    auto it = m_laneZones.find (laneId);
    if (it == m_laneZones.end () || it->second.empty ()) {
        return 1;
    }
    std::cout
        << "[ZONE COUNT] "
        << "Lane=" << laneId
        << " Zones=" << it->second.size()
        << std::endl;
        
    uint32_t bestZoneId = 1;
    double minDistance = 1e9;

    for (const auto &zone : it->second) {
       
        std::cout
            << "[ZONE COUNT] "
            << "Lane=" << laneId
            << " Zones=" << it->second.size()
            << std::endl;
            
        double dist = CalculateDistanceToCenter (zone.centerPoint, vehiclePos);
        
        if (dist < minDistance) {
            minDistance = dist;
            bestZoneId = zone.zoneId;
        }
    }

    return bestZoneId;
}

bool ZoneManager::IsInActiveZone (const Vector &vehiclePos, uint32_t laneId, uint32_t zoneId) const
{
    Zone zone = GetZone (laneId, zoneId);
    return (vehiclePos.x >= zone.activeZoneStartX && vehiclePos.x <= zone.activeZoneEndX);
}

bool ZoneManager::IsInDeadZone (const Vector &vehiclePos, uint32_t laneId, uint32_t zoneId) const
{
    Zone zone = GetZone (laneId, zoneId);
    bool inZoneRange = (vehiclePos.x >= zone.startX && vehiclePos.x <= zone.endX);
    bool inActive = (vehiclePos.x >= zone.activeZoneStartX && vehiclePos.x <= zone.activeZoneEndX);
    return (inZoneRange && !inActive);
}

Zone ZoneManager::GetZone (uint32_t laneId, uint32_t zoneId) const
{
    auto it = m_laneZones.find (laneId);
    if (it != m_laneZones.end ()) {
        for (const auto &zone : it->second) {
            if (zone.zoneId == zoneId) {
                return zone;
            }
        }
    }
    Zone defaultZone;
    defaultZone.laneId = laneId;
    defaultZone.zoneId = zoneId;
    return defaultZone;
}

std::vector<Zone> ZoneManager::GetZonesForLane (uint32_t laneId) const
{
    auto it = m_laneZones.find (laneId);
    if (it != m_laneZones.end ()) {
        return it->second;
    }
    return std::vector<Zone> ();
}

} // namespace ns3
