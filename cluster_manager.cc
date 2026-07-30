#include "cluster_manager.h"
#include <ns3/log.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("ClusterManager");

ClusterManager::ClusterManager ()
    : m_rsuId (0),
      m_rmManager (0.5, 0.5)
{
}

ClusterManager::ClusterManager (uint32_t rsuId)
    : m_rsuId (rsuId),
      m_rmManager (0.5, 0.5)
{
}

void ClusterManager::ProcessHelloNeighMessage (const HelloNeighHeader &header, Time receiveTime)
{
    // Algorithm 2, Lines 1–3: Vehicles send HELLO_NEIGH to RSU
    NeighRecord rec;
    rec.vehicleId = header.GetVehicleId ();
    rec.laneId = header.GetLaneId ();
    rec.position = header.GetPosition ();
    rec.speed = header.GetSpeed ();
    rec.directionType = header.GetDirectionType ();
    rec.neighbors = header.GetNeighborList ();
    rec.receiveTime = receiveTime;

    m_neighRecords[rec.vehicleId] = rec;

    NS_LOG_DEBUG ("RSU " << m_rsuId << " received HELLO_NEIGH from Vehicle " << rec.vehicleId 
                         << " with " << rec.neighbors.size () << " neighbors.");
}

void ClusterManager::PurgeNeighRecords (Time currentTime, Time timeout)
{
    auto it = m_neighRecords.begin ();
    while (it != m_neighRecords.end ()) {
        if ((currentTime - it->second.receiveTime) > timeout) {
            it = m_neighRecords.erase (it);
        } else {
            ++it;
        }
    }
}

std::map<uint32_t, ClusterInfo> ClusterManager::RunChSelectionAlgorithm (
    const ZoneManager &zoneMgr,
    std::map<uint32_t, Vehicle> &vehicles)
{
    // Algorithm 2, Line 4: "for every CH selection Interval each RSU do"
    m_clusters.clear ();

    for (uint32_t laneId = 1; laneId <= 3; ++laneId) {
        std::vector<Zone> zones = zoneMgr.GetZonesForLane (laneId);

        for (const auto &zone : zones) {
            uint32_t clusterKey = (laneId * 10) + zone.zoneId;
            ClusterInfo cluster;
            cluster.clusterId = clusterKey;
            cluster.laneId = laneId;
            cluster.zoneId = zone.zoneId;
            cluster.rsuId = m_rsuId;
            cluster.chVehicleId = 0;
            cluster.chRmValue = 1.0;

            // Algorithm 2, Line 5: Set RM_min = 1
            double rmMin = 1.0;
            uint32_t selectedChId = 0;

            // Collect vehicles located in this zone and specifically within the Active Zone
            std::vector<CandidateData> activeZoneCandidates;
            std::vector<uint32_t> allZoneMembers;

            for (auto &pair : vehicles) {
                Vehicle &v = pair.second;
                
                // Ensure vehicle belongs to current lane
                if (v.GetLaneId () != laneId) {
                    continue;
                }

                Vector pos = v.GetCurrentPosition ();
                uint32_t nearestZoneId = zoneMgr.FindNearestZoneId (pos, laneId);
                
                std::cout
                    << "[ZONE MATCH] "
                    << "Veh=" << v.GetVehicleId()
                    << " x=" << pos.x
                    << " Lane=" << laneId
                    << " nearestZone=" << nearestZoneId
                    << " checkingZone=" << zone.zoneId
                    << std::endl;

                if (nearestZoneId == zone.zoneId) {
                    
                    std::cout
                        << "[ZONE SELECTED] "
                        << "Veh=" << v.GetVehicleId()
                        << " Zone=" << zone.zoneId
                        << std::endl;
                    
                    allZoneMembers.push_back (v.GetVehicleId ());

                    // Algorithm 2, Line 6: "for each Vehicle n within the Active Zone do"
                    bool inActive = zoneMgr.IsInActiveZone (pos, laneId, zone.zoneId);
                    std::cout
                        << "[ACTIVE CHECK] "
                        << "Veh=" << v.GetVehicleId()
                        << " x=" << pos.x
                        << " Zone=" << zone.zoneId
                        << " Active=" << inActive
                        << std::endl;
                    bool inDead = zoneMgr.IsInDeadZone (pos, laneId, zone.zoneId);

                    v.SetAssignedZoneId (zone.zoneId);
                    v.SetInActiveZone (inActive);
                    v.SetInDeadZone (inDead);

                    if (inActive) {
                        cluster.activeZoneVehicleIds.push_back (v.GetVehicleId ());

                        CandidateData cand;
                        cand.vehicleId = v.GetVehicleId ();
                        cand.position = pos;
                        cand.speed = v.GetSpeed ();
                        cand.distanceToCenter = ZoneManager::CalculateDistanceToCenter (zone.centerPoint, pos);
                        cand.calculatedRm = 1.0;

                        activeZoneCandidates.push_back (cand);
                    }
                }
            }

            cluster.memberVehicleIds = allZoneMembers;

            // Algorithm 2, Lines 6–11: Calculate RM_n and select CH with min RM_n
            std::cout << "[CLUSTER DEBUG] activeZoneCandidates = "
                      << activeZoneCandidates.size()
                      << std::endl;
            
            if (!activeZoneCandidates.empty ()) {
                // Calculate RM_n for all active zone candidates using Equation (10)
                m_rmManager.CalculateRmForActiveZoneVehicles (activeZoneCandidates, zone.centerPoint);
                if (m_statsMgr)
                {
                    for (const auto& cand : activeZoneCandidates)
                    {
                        std::cout << "[RM DEBUG] Vehicle " << cand.vehicleId
                                  << " RM = " << cand.calculatedRm << std::endl;
                        
                        m_statsMgr->RecordRM(cand.calculatedRm);
                    }
                }
                // Algorithm 2, Lines 8–10: Find vehicle with min RM
                for (const auto &cand : activeZoneCandidates) {
                    if (cand.calculatedRm < rmMin) {
                        rmMin = cand.calculatedRm;
                        selectedChId = cand.vehicleId; // n -> CH
                    }
                }
            }

            cluster.chVehicleId = selectedChId;
            cluster.chRmValue = rmMin;

            // Update Vehicle modes and Cluster Head assignments according to Algorithm 2 output
            for (uint32_t vId : allZoneMembers) {
                auto vIt = vehicles.find (vId);
                if (vIt != vehicles.end ()) {
                    if (selectedChId != 0 && vId == selectedChId) {
                        // Elected CH
                        vIt->second.SetMode (CH);
                        vIt->second.SetClusterHeadId (selectedChId);
                        vIt->second.SetRelativityMetric (rmMin);
                        NS_LOG_INFO ("Algorithm 2: Vehicle " << vId << " elected as CH for Cluster (Lane " 
                                                             << laneId << ", Zone " << zone.zoneId 
                                                             << ") with RM=" << rmMin);
                    } else {
                        // Cluster Member (CM)
                        vIt->second.SetMode (CM);
                        vIt->second.SetClusterHeadId (selectedChId);
                        
                        // Find candidate RM if in active zone
                        for (const auto &cand : activeZoneCandidates) {
                            if (cand.vehicleId == vId) {
                                vIt->second.SetRelativityMetric (cand.calculatedRm);
                                break;
                            }
                        }
                    }
                }
            }

            m_clusters[clusterKey] = cluster;
        }
    }

    // Algorithm 2, Line 13: "return CH;"
    return m_clusters;
}

bool ClusterManager::GetClusterInfo (uint32_t laneId, uint32_t zoneId, ClusterInfo &cluster) const
{
    uint32_t key = (laneId * 10) + zoneId;
    auto it = m_clusters.find (key);
    if (it != m_clusters.end ()) {
        cluster = it->second;
        return true;
    }
    return false;
}

} // namespace ns3
