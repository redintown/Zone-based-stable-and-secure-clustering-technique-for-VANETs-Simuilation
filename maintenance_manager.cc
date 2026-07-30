#include "maintenance_manager.h"
#include <ns3/log.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("MaintenanceManager");

MaintenanceManager::MaintenanceManager ()
    : m_rsuId (0)
{
}

MaintenanceManager::MaintenanceManager (uint32_t rsuId)
    : m_rsuId (rsuId)
{
}

void MaintenanceManager::CheckRsuChLinksAndMaintain (
    ClusterManager &clusterMgr,
    const ZoneManager &zoneMgr,
    std::map<uint32_t, Vehicle> &vehicles,
    const std::map<uint32_t, Time> &chLastHeardMap,
    Time rsuLinkTimeout,
    Time currentTime)
{
    // Algorithm 3, Lines 1–3:
    // if RSU has no link with CH then
    //     Call algorithm 2;
    // end if
    bool reselectionNeeded = false;
    const auto &clusters = clusterMgr.GetAllClusters ();

    for (const auto &pair : clusters) {
        const ClusterInfo &cluster = pair.second;
        uint32_t chId = cluster.chVehicleId;

        if (chId != 0) {
            auto it = chLastHeardMap.find (chId);
            if (it == chLastHeardMap.end () || (currentTime - it->second) > rsuLinkTimeout) {
                NS_LOG_INFO ("Algorithm 3 [RSU " << m_rsuId << "]: Lost link with CH " << chId 
                                                 << " in cluster " << cluster.clusterId 
                                                 << ". Triggering Algorithm 2.");
                reselectionNeeded = true;
                break;
            }
        }
    }

    if (reselectionNeeded) {
        clusterMgr.RunChSelectionAlgorithm (zoneMgr, vehicles);
    }
}

bool MaintenanceManager::HandleChLostCm (Vehicle &chVehicle, uint32_t cmVehicleId, uint32_t &notifiedRsuId)
{
    // Algorithm 3, Lines 4–7:
    // if CH has no access to reach CM then
    //     Remove the CM from one-hop neighbor table of CH;
    //     Notify RSU;
    // end if
    chVehicle.RemoveNeighbor (cmVehicleId);
    notifiedRsuId = m_rsuId;

    NS_LOG_INFO ("Algorithm 3 [CH " << chVehicle.GetVehicleId () << "]: Lost CM " << cmVehicleId 
                                    << ". Erased from OHN table and notified RSU " << m_rsuId << ".");
    return true;
}

VehicleMode MaintenanceManager::HandleCmLostCh (
    Vehicle &cmVehicle,
    const std::map<uint32_t, ChSignalInfo> &chSignals,
    std::map<uint32_t, Vehicle> &vehicles)
{
    // Algorithm 3, Lines 8–16:
    // if a CM has no link with CH then
    //     if it receives signals from multiple CHs then
    //         Connect to the cluster whose CH's signal strength is strongest;
    //         CH checks the unique ID of the CM;
    //     else
    //         Notify RSU;
    //         Node will act as CH;
    //     end if
    // end if

    if (!chSignals.empty ()) {
        // Find CH with maximum signal strength
        uint32_t bestChId = 0;
        double maxSignal = -999.0;

        for (const auto &pair : chSignals) {
            if (pair.second.signalStrengthDbm > maxSignal) {
                maxSignal = pair.second.signalStrengthDbm;
                bestChId = pair.first;
            }
        }

        if (bestChId != 0) {
            cmVehicle.SetMode (CM);
            cmVehicle.SetClusterHeadId (bestChId);

            // CH checks unique ID of CM (Line 11)
            auto chIt = vehicles.find (bestChId);
            if (chIt != vehicles.end ()) {
                NeighborEntry entry (
                    cmVehicle.GetVehicleId (),
                    cmVehicle.GetLaneId (),
                    cmVehicle.GetRoadId (),
                    cmVehicle.GetCurrentPosition (),
                    cmVehicle.GetSpeed (),
                    cmVehicle.GetDirectionType (),
                    Simulator::Now ()
                );
                chIt->second.AddOrUpdateNeighbor (entry);
            }

            NS_LOG_INFO ("Algorithm 3 [CM " << cmVehicle.GetVehicleId () << "]: Connected to strongest CH " 
                                            << bestChId << " (Signal: " << maxSignal << " dBm).");
            return CM;
        }
    }

    // Line 12–15: No strong signal from any CH -> Notify RSU & Act as CH
    cmVehicle.SetMode (CH);
    cmVehicle.SetClusterHeadId (cmVehicle.GetVehicleId ());
    NS_LOG_INFO ("Algorithm 3 [Vehicle " << cmVehicle.GetVehicleId () 
                                         << "]: No valid CH signals. Notified RSU and acting as CH.");
    return CH;
}

void MaintenanceManager::HandleNewVehicleEntry (
    Vehicle &newVehicle,
    const std::vector<uint32_t> &nearbyChs,
    ClusterManager &clusterMgr,
    const ZoneManager &zoneMgr,
    std::map<uint32_t, Vehicle> &vehicles)
{
    // Section 3.4 (C): New vehicle enters network
    // At first tries to connect to nearby CH via DSRC
    bool connectedToCh = false;

    for (uint32_t chId : nearbyChs) {
        auto chIt = vehicles.find (chId);
        if (chIt != vehicles.end () && chIt->second.GetLaneId () == newVehicle.GetLaneId ()) {
            newVehicle.SetMode (CM);
            newVehicle.SetClusterHeadId (chId);

            NeighborEntry entry (
                newVehicle.GetVehicleId (),
                newVehicle.GetLaneId (),
                newVehicle.GetRoadId (),
                newVehicle.GetCurrentPosition (),
                newVehicle.GetSpeed (),
                newVehicle.GetDirectionType (),
                Simulator::Now ()
            );
            chIt->second.AddOrUpdateNeighbor (entry);
            connectedToCh = true;

            NS_LOG_INFO ("New Vehicle " << newVehicle.GetVehicleId () << " joined existing CH " << chId);
            break;
        }
    }

    // Upon failure, broadcast to RSU -> RSU assists connection or lets vehicle act as CH
    if (!connectedToCh) {
        clusterMgr.RunChSelectionAlgorithm (zoneMgr, vehicles);
        if (newVehicle.GetMode () == UNCERTAIN_MODE || newVehicle.GetMode () == CHA) {
            newVehicle.SetMode (CH);
            newVehicle.SetClusterHeadId (newVehicle.GetVehicleId ());
            NS_LOG_INFO ("New Vehicle " << newVehicle.GetVehicleId () << " created new cluster as CH.");
        }
    }
}

} // namespace ns3
