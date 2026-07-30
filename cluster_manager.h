#ifndef CLUSTER_MANAGER_H
#define CLUSTER_MANAGER_H

#include "vehicle.h"
#include "zone_manager.h"
#include "rm_manager.h"
#include "hello_message.h"
#include "statistics_manager.h"
#include <ns3/nstime.h>
#include <ns3/vector.h>
#include <vector>
#include <map>
#include <cstdint>

namespace ns3 {

/**
 * @brief Structure representing a Cluster in a specific lane and zone.
 */
struct ClusterInfo {
    uint32_t clusterId{0};
    uint32_t laneId{0};
    uint32_t zoneId{0};
    uint32_t rsuId{0};
    uint32_t chVehicleId{0};
    double chRmValue{1.0};
    std::vector<uint32_t> memberVehicleIds;
    std::vector<uint32_t> activeZoneVehicleIds;
};

/**
 * @brief Manages Cluster Head (CH) Selection exactly according to Algorithm 2.
 * Executes on RSU for every CH selection interval.
 */
class ClusterManager {
public:
    ClusterManager();
    explicit ClusterManager(uint32_t rsuId);
    ~ClusterManager() = default;
    void SetStatisticsManager(StatisticsManager* statsMgr)
    {
        m_statsMgr = statsMgr;
    }
    /**
     * @brief Sets the parent RSU ID.
     */
    void SetRsuId(uint32_t rsuId) { m_rsuId = rsuId; }

    /**
     * @brief Gets the parent RSU ID.
     */
    uint32_t GetRsuId() const { return m_rsuId; }

    /**
     * @brief Sets RM weight parameters A and B (A + B = 1, default 0.5 each).
     */
    void SetRmWeights(double a, double b) { m_rmManager.SetWeights(a, b); }

    /**
     * @brief Processes HELLO_NEIGH message sent by a vehicle to RSU (Algorithm 2, Lines 1–3).
     * 
     * @param header Received HelloNeighHeader
     * @param receiveTime Time of reception
     */
    void ProcessHelloNeighMessage(const HelloNeighHeader& header, Time receiveTime);

    /**
     * @brief Executes Algorithm 2: CH selection mechanism at RSU for each cluster.
     * 
     * Lines 4–13:
     * - For each zone/cluster in Active Zone:
     *   - Set RM_min = 1
     *   - Calculate RM_n for each vehicle n in Active Zone via Eq. (10)
     *   - If RM_n < RM_min, update RM_min = RM_n and elect n -> CH
     * - Updates vehicle state modes (CH vs CM) and cluster assignments.
     * 
     * @param zoneMgr Reference to ZoneManager with predefined active zones
     * @param vehicles Map of all vehicle states managed in network
     * @return Map of cluster key (laneId * 10 + zoneId) to updated ClusterInfo
     */
    std::map<uint32_t, ClusterInfo> RunChSelectionAlgorithm(
        const ZoneManager& zoneMgr,
        std::map<uint32_t, Vehicle>& vehicles);

    /**
     * @brief Gets information for a specific cluster by lane and zone.
     */
    bool GetClusterInfo(uint32_t laneId, uint32_t zoneId, ClusterInfo& cluster) const;

    /**
     * @brief Gets all active clusters managed by this RSU.
     */
    const std::map<uint32_t, ClusterInfo>& GetAllClusters() const { return m_clusters; }

    /**
     * @brief Clears stale HELLO_NEIGH records received prior to timeout.
     */
    void PurgeNeighRecords(Time currentTime, Time timeout);

private:
    /**
     * @brief Structure to store HELLO_NEIGH message contents on RSU.
     */
    struct NeighRecord {
        uint32_t vehicleId{0};
        uint32_t laneId{0};
        Vector position{0.0, 0.0, 0.0};
        double speed{0.0};
        uint8_t directionType{1};
        std::vector<uint32_t> neighbors;
        Time receiveTime{Seconds(0)};
    };

    uint32_t m_rsuId{0};
    RmManager m_rmManager;
    StatisticsManager* m_statsMgr{nullptr};
    std::map<uint32_t, NeighRecord> m_neighRecords; // Vehicle ID -> NeighRecord
    std::map<uint32_t, ClusterInfo> m_clusters;     // Cluster Key -> ClusterInfo
};

} // namespace ns3

#endif // CLUSTER_MANAGER_H

