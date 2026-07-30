#ifndef SECURITY_MANAGER_H
#define SECURITY_MANAGER_H

#include "vehicle.h"
#include <ns3/header.h>
#include <ns3/type-id.h>
#include <ns3/buffer.h>
#include <ns3/vector.h>
#include <ns3/nstime.h>
#include <map>
#include <vector>
#include <cstdint>
#include <cstring>
#include <iostream>

namespace ns3 {

/**
 * @brief ICMP Control Message Header used for security monitoring and mitigation in VANETs.
 * As described in Section 3.5:
 * - ICMP Echo Request: Broadcasted when 2+ vehicles claim same ID.
 * - ICMP Echo Reply: Sent by legitimate vehicles to confirm identity.
 * - ICMP Redirect / Router Advertisement: Used to direct traffic away from rogue/malicious nodes.
 */
class IcmpVanetHeader : public Header {
public:
    enum IcmpType {
        ICMP_ECHO_REQUEST = 8,
        ICMP_ECHO_REPLY = 0,
        ICMP_REDIRECT = 5,
        ICMP_ROUTER_ADVERTISEMENT = 9
    };

    IcmpVanetHeader();
    ~IcmpVanetHeader() override = default;

    static TypeId GetTypeId(void);
    TypeId GetInstanceTypeId(void) const override;
    uint32_t GetSerializedSize(void) const override;
    void Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;
    void Print(std::ostream &os) const override;

    uint8_t GetIcmpType() const { return m_type; }
    void SetIcmpType(uint8_t type) { m_type = type; }

    uint8_t GetIcmpCode() const { return m_code; }
    void SetIcmpCode(uint8_t code) { m_code = code; }

    uint32_t GetTargetVehicleId() const { return m_targetVehicleId; }
    void SetTargetVehicleId(uint32_t id) { m_targetVehicleId = id; }

    uint32_t GetRsuId() const { return m_rsuId; }
    void SetRsuId(uint32_t id) { m_rsuId = id; }

    uint64_t GetTimestampUs() const { return m_timestampUs; }
    void SetTimestampUs(uint64_t ts) { m_timestampUs = ts; }

private:
    uint8_t m_type{ICMP_ECHO_REQUEST};
    uint8_t m_code{0};
    uint32_t m_targetVehicleId{0};
    uint32_t m_rsuId{0};
    uint64_t m_timestampUs{0};
};

/**
 * @brief Structure for tracking vehicle identity and neighbor reports according to Algorithm 4.
 */
struct VehicleReport {
    uint32_t realVehicleId{0};       // Actual hardware/node ID
    uint32_t claimedVehicleId{0};    // ID broadcasted in packet (V_ID)
    uint32_t reportingNeighborId{0}; // Vehicle that received and forwarded packet
    Vector location{0.0, 0.0, 0.0};
    Time timestamp{Seconds(0)};
};

/**
 * @brief Implements Algorithm 4: Impersonation Attack Detection & Security Management.
 * 
 * Section 3.5 & Algorithm 4:
 * 1. Vehicles update neighbor tables with packet P sender ID, timestamp, and location.
 * 2. If same V_ID is claimed by multiple different neighbors or locations change unnaturally:
 *    - Node marked as Suspect[Malicious].
 * 3. RSU switches to Promiscuous Mode to monitor suspect traffic.
 * 4. RSU broadcasts ICMP messages (Echo Request, Redirect) to isolate the attacker.
 */
class SecurityManager {
public:
    SecurityManager();
    explicit SecurityManager(uint32_t rsuId);
    ~SecurityManager() = default;

    void SetRsuId(uint32_t rsuId) { m_rsuId = rsuId; }
    uint32_t GetRsuId() const { return m_rsuId; }

    bool IsPromiscuousMode() const { return m_promiscuousMode; }
    void SetPromiscuousMode(bool enable) { m_promiscuousMode = enable; }

    /**
     * @brief Algorithm 4, Lines 3–10: Neighbor Update (Packet P).
     * Called by vehicles when receiving packets. Updates neighbor timestamp and location.
     */
    void ProcessNeighborUpdate(
        uint32_t reportingNodeId,
        uint32_t claimedSenderId,
        uint32_t actualSenderId,
        Vector position,
        Time timestamp);

    /**
     * @brief Algorithm 4, Lines 11–16: Detect Impersonation Attack.
     * Checks if V_ID has multiple different neighbors/locations comparing against previous timestamp.
     * 
     * @param currentTime Current simulation time
     * @param suspectList Output vector of suspected malicious claimed IDs
     * @return true if impersonation attack detected
     */
    bool DetectImpersonationAttack(Time currentTime, std::vector<uint32_t>& suspectList);

    /**
     * @brief Algorithm 4, Lines 17–20: Promiscuous Change Mode & ICMP Monitoring.
     * RSU monitors suspect list, issues ICMP Echo Requests, marks malicious node, and isolates it.
     * 
     * @param suspectVehicleId Claimed ID suspected of impersonation
     * @param currentTime Current simulation time
     * @param icmpHeader Output ICMP header generated for broadcast
     */
    void IssueIcmpMonitoringAndIsolate(
        uint32_t suspectVehicleId,
        Time currentTime,
        IcmpVanetHeader& icmpHeader);

    /**
     * @brief Checks if a vehicle claimed ID is marked as confirmed malicious.
     */
    bool IsMalicious(uint32_t claimedVehicleId) const;

    /**
     * @brief Gets all confirmed malicious vehicle IDs.
     */
    const std::vector<uint32_t>& GetMaliciousList() const { return m_maliciousList; }

    /**
     * @brief Clears past reports prior to timeout window.
     */
    void PurgeOldReports(Time currentTime, Time windowSize);

private:
    uint32_t m_rsuId{0};
    bool m_promiscuousMode{false};

    // Claimed ID -> list of received reports from different physical neighbors
    std::map<uint32_t, std::vector<VehicleReport>> m_neighborReports;
    
    // Suspect ID -> timestamp when marked suspect
    std::map<uint32_t, Time> m_suspectList;

    // Confirmed isolated malicious IDs
    std::vector<uint32_t> m_maliciousList;
};

} // namespace ns3

#endif // SECURITY_MANAGER_H
