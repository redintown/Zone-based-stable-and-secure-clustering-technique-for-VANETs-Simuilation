#ifndef VEHICLE_H
#define VEHICLE_H

#include <ns3/vector.h>
#include <ns3/nstime.h>
#include <ns3/ipv4-address.h>
#include <ns3/mac48-address.h>
#include <cmath>
#include <map>
#include <vector>
#include <cstdint>

namespace ns3 {

/**
 * @brief Vehicle Operational Modes as defined in the research paper.
 * - UNCERTAIN_MODE (UM): Initial mode where vehicles are positioned randomly without interconnection.
 * - CHA: Cluster Head Applicant mode.
 * - CH: Cluster Head mode.
 * - CM: Cluster Member mode.
 * - MONITOR_MODE: Vehicle monitoring mode during security analysis.
 */
enum VehicleMode {
    UNCERTAIN_MODE = 0,
    CHA = 1,
    CH = 2,
    CM = 3,
    MONITOR_MODE = 4
};

/**
 * @brief Structure representing an entry in the One-Hop Neighbor Table (OHN).
 * Stores geographical, mobility, and temporal metrics received via HELLO messages.
 */
struct NeighborEntry {
    uint32_t vehicleId{0};
    uint32_t laneId{0};
    uint32_t roadId{0};
    Vector position{0.0, 0.0, 0.0};
    double speed{0.0};
    uint8_t directionType{1};
    Time lastTimestamp{Seconds(0)};
    Vector lastLocation{0.0, 0.0, 0.0};
    
    NeighborEntry() = default;
    
    NeighborEntry(uint32_t id, uint32_t lane, uint32_t road, Vector pos, double spd, uint8_t dir, Time ts)
        : vehicleId(id), laneId(lane), roadId(road), position(pos), speed(spd), 
          directionType(dir), lastTimestamp(ts), lastLocation(pos) {}
};

/**
 * @brief Complete Vehicle state representation required for zone-based clustering and security mechanisms.
 */
class Vehicle {
public:
    Vehicle() 
        : m_vehicleId(0),
          m_claimedId(0),
          m_laneId(1),
          m_roadId(1),
          m_currentPosition(0.0, 0.0, 0.0),
          m_targetPosition(0.0, 0.0, 0.0),
          m_speed(0.0),
          m_directionAngle(0.0),
          m_directionType(1),
          m_mode(UNCERTAIN_MODE),
          m_relativityMetric(1.0),
          m_distanceToZoneCenter(0.0),
          m_assignedZoneId(0),
          m_inDeadZone(false),
          m_inActiveZone(false),
          m_clusterHeadId(0),
          m_isMalicious(false),
          m_isSuspect(false),
          m_suspectTimestamp(Seconds(0))
    {}

    Vehicle(uint32_t id, uint32_t lane, uint32_t road)
        : m_vehicleId(id),
          m_claimedId(id),
          m_laneId(lane),
          m_roadId(road),
          m_currentPosition(0.0, 0.0, 0.0),
          m_targetPosition(0.0, 0.0, 0.0),
          m_speed(0.0),
          m_directionAngle(0.0),
          m_directionType(1),
          m_mode(UNCERTAIN_MODE),
          m_relativityMetric(1.0),
          m_distanceToZoneCenter(0.0),
          m_assignedZoneId(0),
          m_inDeadZone(false),
          m_inActiveZone(false),
          m_clusterHeadId(0),
          m_isMalicious(false),
          m_isSuspect(false),
          m_suspectTimestamp(Seconds(0))
    {}

    // Identification Getters and Setters
    uint32_t GetVehicleId() const { return m_vehicleId; }
    void SetVehicleId(uint32_t id) { m_vehicleId = id; }

    uint32_t GetClaimedId() const { return m_claimedId; }
    void SetClaimedId(uint32_t id) { m_claimedId = id; }

    uint32_t GetLaneId() const { return m_laneId; }
    void SetLaneId(uint32_t lane) { m_laneId = lane; }

    uint32_t GetRoadId() const { return m_roadId; }
    void SetRoadId(uint32_t road) { m_roadId = road; }

    Ipv4Address GetIpAddress() const { return m_ipAddress; }
    void SetIpAddress(Ipv4Address ip) { m_ipAddress = ip; }

    Mac48Address GetMacAddress() const { return m_macAddress; }
    void SetMacAddress(Mac48Address mac) { m_macAddress = mac; }

    // Mobility Getters and Setters
    Vector GetCurrentPosition() const { return m_currentPosition; }
    void SetCurrentPosition(const Vector& pos) { 
        m_currentPosition = pos; 
        UpdateDirection();
    }

    Vector GetTargetPosition() const { return m_targetPosition; }
    void SetTargetPosition(const Vector& target) { 
        m_targetPosition = target; 
        UpdateDirection();
    }

    double GetSpeed() const { return m_speed; }
    void SetSpeed(double speed) { m_speed = speed; }

    double GetDirectionAngle() const { return m_directionAngle; }
    uint8_t GetDirectionType() const { return m_directionType; }

    /**
     * @brief Calculates direction angle theta_n and direction quadrant D_n.
     * Equations (2) & (3):
     * theta_n = tan^-1((y'_n - y_n) / (x'_n - x_n))
     * D_n in {1: [0, 90 deg), 2: [90, 180 deg), 3: [180, 270 deg), 4: [270, 360 deg)}
     */
    void UpdateDirection() {
        double dx = m_targetPosition.x - m_currentPosition.x;
        double dy = m_targetPosition.y - m_currentPosition.y;
        
        double rad = std::atan2(dy, dx);
        double deg = rad * (180.0 / M_PI);
        if (deg < 0.0) {
            deg += 360.0;
        }
        
        m_directionAngle = deg;

        if (deg >= 0.0 && deg < 90.0) {
            m_directionType = 1;
        } else if (deg >= 90.0 && deg < 180.0) {
            m_directionType = 2;
        } else if (deg >= 180.0 && deg < 270.0) {
            m_directionType = 3;
        } else {
            m_directionType = 4;
        }
    }

    // Operational Mode
    VehicleMode GetMode() const { return m_mode; }
    void SetMode(VehicleMode mode) { m_mode = mode; }

    // Clustering Metrics
    double GetRelativityMetric() const { return m_relativityMetric; }
    void SetRelativityMetric(double rm) { m_relativityMetric = rm; }

    double GetDistanceToZoneCenter() const { return m_distanceToZoneCenter; }
    void SetDistanceToZoneCenter(double dist) { m_distanceToZoneCenter = dist; }

    uint32_t GetAssignedZoneId() const { return m_assignedZoneId; }
    void SetAssignedZoneId(uint32_t zoneId) { m_assignedZoneId = zoneId; }

    bool IsInDeadZone() const { return m_inDeadZone; }
    void SetInDeadZone(bool dead) { m_inDeadZone = dead; }

    bool IsInActiveZone() const { return m_inActiveZone; }
    void SetInActiveZone(bool active) { m_inActiveZone = active; }

    uint32_t GetClusterHeadId() const { return m_clusterHeadId; }
    void SetClusterHeadId(uint32_t chId) { m_clusterHeadId = chId; }

    // One-Hop Neighbor Table (OHN) Management
    const std::map<uint32_t, NeighborEntry>& GetNeighborTable() const { return m_neighborTable; }
    
    void AddOrUpdateNeighbor(const NeighborEntry& entry) {
        m_neighborTable[entry.vehicleId] = entry;
    }

    void RemoveNeighbor(uint32_t vehicleId) {
        m_neighborTable.erase(vehicleId);
    }

    void ClearNeighborTable() {
        m_neighborTable.clear();
    }

    uint32_t GetNeighborCount() const {
        return static_cast<uint32_t>(m_neighborTable.size());
    }

    // Security & Impersonation Attack Tracking
    bool IsMalicious() const { return m_isMalicious; }
    void SetMalicious(bool malicious) { m_isMalicious = malicious; }

    bool IsSuspect() const { return m_isSuspect; }
    void SetSuspect(bool suspect) { m_isSuspect = suspect; }

    Time GetSuspectTimestamp() const { return m_suspectTimestamp; }
    void SetSuspectTimestamp(Time ts) { m_suspectTimestamp = ts; }

private:
    uint32_t m_vehicleId;
    uint32_t m_claimedId;
    uint32_t m_laneId;
    uint32_t m_roadId;
    
    Ipv4Address m_ipAddress;
    Mac48Address m_macAddress;

    Vector m_currentPosition;
    Vector m_targetPosition;
    double m_speed;
    double m_directionAngle;
    uint8_t m_directionType;

    VehicleMode m_mode;

    double m_relativityMetric;
    double m_distanceToZoneCenter;
    uint32_t m_assignedZoneId;
    bool m_inDeadZone;
    bool m_inActiveZone;
    uint32_t m_clusterHeadId;

    std::map<uint32_t, NeighborEntry> m_neighborTable;

    bool m_isMalicious;
    bool m_isSuspect;
    Time m_suspectTimestamp;
};

} // namespace ns3

#endif // VEHICLE_H
