#ifndef ZONE_MANAGER_H
#define ZONE_MANAGER_H

#include <ns3/vector.h>
#include <ns3/core-module.h>
#include <vector>
#include <map>
#include <cmath>
#include <cstdint>

namespace ns3 {

/**
 * @brief Structure representing a Zone in a specific lane managed by an RSU.
 * According to Section 3.1:
 * - Each RSU coverage region (900 m) is divided into 3 equal fixed zones.
 * - Each zone has a fixed center point (x_c, y_c).
 * - Each zone is divided into Dead Zones (boundary regions) and Active Zone (center region).
 */
struct Zone {
    uint32_t zoneId{0};          // Zone index (1, 2, or 3)
    uint32_t laneId{0};          // Lane ID (1, 2, or 3)
    uint32_t rsuId{0};           // Associated RSU ID
    Vector centerPoint{0,0,0};   // Fixed center point (x_c, y_c)
    double startX{0.0};          // Zone start boundary (x)
    double endX{0.0};            // Zone end boundary (x)
    double laneY{0.0};           // Lane Y coordinate
    double deadZoneWidth{0.0};   // Dead zone width calculated via Eq. (1)
    double activeZoneStartX{0.0};// Active zone start boundary
    double activeZoneEndX{0.0};  // Active zone end boundary
    double activeZoneLength{0.0};// Length of active zone
};

/**
 * @brief Manages Zone division, Active Zones, Dead Zones, and Fixed Center Points
 * exactly as specified in Section 3.1 and Equations (1) & (4).
 */
class ZoneManager {
public:
    ZoneManager();
    ~ZoneManager() = default;

    /**
     * @brief Initializes RSU zonal division for 3 lanes and 3 zones per lane.
     * @param rsuId ID of the RSU
     * @param rsuPos Center position of the RSU
     * @param rsuCoverage Total coverage range of RSU (default 900.0 m)
     * @param numZones Number of zones per lane (default 3)
     */
    void InitializeRsuZones(uint32_t rsuId, const Vector& rsuPos, double rsuCoverage = 900.0, uint32_t numZones = 3);

    /**
     * @brief Calculates Dead Zone width using Eq. (1):
     * Dead Zone = (S_L,avg * t_HELLO) / 4
     * @param laneAvgSpeed Average speed of the lane (m/s)
     * @param tHello HELLO broadcast interval (s, default 1.0 s)
     * @return Dead zone boundary width in meters
     */
    static double CalculateDeadZoneWidth(double laneAvgSpeed, double tHello = 1.0);

    /**
     * @brief Gets average lane speed based on Section 3.1 speed limits:
     * Lane 1 (10-20 m/s): Avg = 15.0 m/s
     * Lane 2 (21-30 m/s): Avg = 25.5 m/s
     * Lane 3 (31-40 m/s): Avg = 35.5 m/s
     */
    static double GetLaneAverageSpeed(uint32_t laneId);

    /**
     * @brief Calculates Euclidean distance between vehicle and RSU using Eq. (4):
     * d_{r,n} = sqrt((x_r - x_n)^2 + (y_r - y_n)^2)
     */
    static double CalculateDistanceToRsu(const Vector& rsuPos, const Vector& vehiclePos);

    /**
     * @brief Calculates Euclidean distance between vehicle and fixed center point of a zone.
     * d_{c,n} = sqrt((x_c - x_n)^2 + (y_c - y_n)^2)
     */
    static double CalculateDistanceToCenter(const Vector& centerPos, const Vector& vehiclePos);

    /**
     * @brief Finds the zone ID in a given lane where distance to zone center point is minimum.
     * According to Algorithm 1 (lines 11-13).
     */
    uint32_t FindNearestZoneId(const Vector& vehiclePos, uint32_t laneId) const;

    /**
     * @brief Checks if a vehicle position lies within the Active Zone of a specific zone.
     */
    bool IsInActiveZone(const Vector& vehiclePos, uint32_t laneId, uint32_t zoneId) const;

    /**
     * @brief Checks if a vehicle position lies within the Dead Zone of a specific zone.
     */
    bool IsInDeadZone(const Vector& vehiclePos, uint32_t laneId, uint32_t zoneId) const;

    /**
     * @brief Gets the Zone structure for a given lane and zone ID.
     */
    Zone GetZone(uint32_t laneId, uint32_t zoneId) const;

    /**
     * @brief Gets all zones for a specific lane.
     */
    std::vector<Zone> GetZonesForLane(uint32_t laneId) const;

    /**
     * @brief Sets custom Y coordinate for a specific lane.
     */
    void SetLaneYCoordinate(uint32_t laneId, double y);

    /**
     * @brief Gets Y coordinate for a specific lane.
     */
    double GetLaneYCoordinate(uint32_t laneId) const;

private:
    uint32_t m_rsuId{0};
    Vector m_rsuPosition{0.0, 0.0, 0.0};
    double m_rsuCoverage{900.0};
    uint32_t m_numZonesPerLane{3};

    std::map<uint32_t, std::vector<Zone>> m_laneZones;
    std::map<uint32_t, double> m_laneYCoords;
};

} // namespace ns3

#endif // ZONE_MANAGER_H
