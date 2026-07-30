#ifndef MAINTENANCE_MANAGER_H
#define MAINTENANCE_MANAGER_H

#include "vehicle.h"
#include "cluster_manager.h"
#include "zone_manager.h"
#include <ns3/nstime.h>
#include <map>
#include <vector>
#include <cstdint>

namespace ns3 {

/**
 * @brief Structure representing signal metrics received by a CM from neighboring CHs.
 */
struct ChSignalInfo {
    uint32_t chVehicleId{0};
    double signalStrengthDbm{-100.0};
    Time lastReceivedTime{Seconds(0)};
};

/**
 * @brief Implements Algorithm 3: Maintenance and Re-creation Algorithm exactly as specified in Section 3.4.
 * 
 * Handles 3 main cases:
 * (A) No link between CH and CM (Algorithm 3, Lines 4–16)
 * (B) No link between CH and RSU (Algorithm 3, Lines 1–3)
 * (C) New vehicle enters network (Section 3.4 C)
 */
class MaintenanceManager {
public:
    MaintenanceManager();
    explicit MaintenanceManager(uint32_t rsuId);
    ~MaintenanceManager() = default;

    /**
     * @brief Sets parent RSU ID.
     */
    void SetRsuId(uint32_t rsuId) { m_rsuId = rsuId; }

    /**
     * @brief Gets parent RSU ID.
     */
    uint32_t GetRsuId() const { return m_rsuId; }

    /**
     * @brief Executes Algorithm 3 for RSU-level link checks.
     * 
     * Lines 1–3:
     * if RSU has no link with CH then
     *     Call algorithm 2;
     * end if
     * 
     * @param clusterMgr Reference to ClusterManager on RSU
     * @param zoneMgr Reference to ZoneManager on RSU
     * @param vehicles Map of active vehicles
     * @param chLastHeardMap Map tracking last time RSU received signal/packet from each CH ID
     * @param rsuLinkTimeout Maximum allowed silence before declaring loss of RSU-CH link
     * @param currentTime Current simulation time
     */
    void CheckRsuChLinksAndMaintain(
        ClusterManager& clusterMgr,
        const ZoneManager& zoneMgr,
        std::map<uint32_t, Vehicle>& vehicles,
        const std::map<uint32_t, Time>& chLastHeardMap,
        Time rsuLinkTimeout,
        Time currentTime);

    /**
     * @brief Executes CH-side maintenance when CH loses access to reach a CM (Algorithm 3, Lines 4–7).
     * 
     * if CH has no access to reach CM then
     *     Remove CM from one-hop neighbor table of CH;
     *     Notify RSU;
     * end if
     * 
     * @param chVehicle Reference to CH vehicle
     * @param cmVehicleId ID of CM that is unreachable
     * @param notifiedRsuId Output parameter set to RSU ID notified
     * @return true if CM was removed and RSU notified
     */
    bool HandleChLostCm(Vehicle& chVehicle, uint32_t cmVehicleId, uint32_t& notifiedRsuId);

    /**
     * @brief Executes CM-side maintenance when CM loses link with its CH (Algorithm 3, Lines 8–16).
     * 
     * if a CM has no link with CH then
     *     if it receives signals from multiple CHs then
     *         Connect to cluster whose CH's signal strength is strongest;
     *         CH checks unique ID of CM;
     *     else
     *         Notify RSU;
     *         Node will act as CH;
     *     end if
     * end if
     * 
     * @param cmVehicle Reference to CM vehicle
     * @param chSignals Map of CH Vehicle IDs to received signal info (ChSignalInfo)
     * @param vehicles Map of all vehicles in network
     * @return Transformed mode of the node (CM or CH)
     */
    VehicleMode HandleCmLostCh(
        Vehicle& cmVehicle,
        const std::map<uint32_t, ChSignalInfo>& chSignals,
        std::map<uint32_t, Vehicle>& vehicles);

    /**
     * @brief Handles entry of a new vehicle into the network according to Section 3.4 (C).
     * 
     * "While entering the network, at first, a vehicle tries to connect to the nearby located cluster 
     * by sending HELLO_MSG to CH as a request using DSRC. Upon failure, a vehicle will broadcast a 
     * HELLO MSG to RSU. Further, RSU will assist the newcomer to connect with the persisting cluster 
     * or act as CH and create a new cluster."
     * 
     * @param newVehicle Reference to newcomer vehicle
     * @param nearbyChs List of nearby active CH IDs reachable via DSRC
     * @param clusterMgr Reference to ClusterManager
     * @param zoneMgr Reference to ZoneManager
     * @param vehicles Map of active vehicles
     */
    void HandleNewVehicleEntry(
        Vehicle& newVehicle,
        const std::vector<uint32_t>& nearbyChs,
        ClusterManager& clusterMgr,
        const ZoneManager& zoneMgr,
        std::map<uint32_t, Vehicle>& vehicles);

private:
    uint32_t m_rsuId{0};
};

} // namespace ns3

#endif // MAINTENANCE_MANAGER_H
