#include "security_manager.h"
#include <ns3/log.h>
#include <cmath>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("SecurityManager");

// ------------------------------------------------------------------
// IcmpVanetHeader Implementation
// ------------------------------------------------------------------

IcmpVanetHeader::IcmpVanetHeader ()
    : m_type (ICMP_ECHO_REQUEST),
      m_code (0),
      m_targetVehicleId (0),
      m_rsuId (0),
      m_timestampUs (0)
{
}

TypeId IcmpVanetHeader::GetTypeId (void)
{
    static TypeId tid = TypeId ("ns3::IcmpVanetHeader")
        .SetParent<Header> ()
        .SetGroupName ("Vanet")
        .AddConstructor<IcmpVanetHeader> ();
    return tid;
}

TypeId IcmpVanetHeader::GetInstanceTypeId (void) const
{
    return GetTypeId ();
}

uint32_t IcmpVanetHeader::GetSerializedSize (void) const
{
    return 18;
}

void IcmpVanetHeader::Serialize (Buffer::Iterator start) const
{
    start.WriteU8 (m_type);
    start.WriteU8 (m_code);
    start.WriteHtonU32 (m_targetVehicleId);
    start.WriteHtonU32 (m_rsuId);
    start.WriteHtonU64 (m_timestampUs);
}

uint32_t IcmpVanetHeader::Deserialize (Buffer::Iterator start)
{
    m_type = start.ReadU8 ();
    m_code = start.ReadU8 ();
    m_targetVehicleId = start.ReadNtohU32 ();
    m_rsuId = start.ReadNtohU32 ();
    m_timestampUs = start.ReadNtohU64 ();
    return GetSerializedSize ();
}

void IcmpVanetHeader::Print (std::ostream &os) const
{
    os << "IcmpType=" << static_cast<uint32_t> (m_type)
       << " Code=" << static_cast<uint32_t> (m_code)
       << " TargetVehicleId=" << m_targetVehicleId
       << " RsuId=" << m_rsuId
       << " TimestampUs=" << m_timestampUs;
}

// ------------------------------------------------------------------
// SecurityManager Implementation (Algorithm 4)
// ------------------------------------------------------------------

SecurityManager::SecurityManager ()
    : m_rsuId (0),
      m_promiscuousMode (false)
{
}

SecurityManager::SecurityManager (uint32_t rsuId)
    : m_rsuId (rsuId),
      m_promiscuousMode (false)
{
}

void SecurityManager::ProcessNeighborUpdate (
    uint32_t reportingNodeId,
    uint32_t claimedSenderId,
    uint32_t actualSenderId,
    Vector position,
    Time timestamp)
{
    // Algorithm 4, Lines 4–10:
    // Neighbor Update (Packet P)
    // if (P.Sender is not Neighbor) then Neighbor.Add (Sender)
    // Neighbor [sender]. time stamp = p. time stamp
    // Neighbor [sender]. location = p.location
    VehicleReport report;
    report.realVehicleId = actualSenderId;
    report.claimedVehicleId = claimedSenderId;
    report.reportingNeighborId = reportingNodeId;
    report.location = position;
    report.timestamp = timestamp;

    m_neighborReports[claimedSenderId].push_back (report);

    NS_LOG_DEBUG ("Algorithm 4: Recorded report for Claimed VID " << claimedSenderId 
                                  << " from Node " << reportingNodeId 
                                  << " (Actual Node " << actualSenderId 
                                  << ") at pos (" << position.x << "," << position.y << ")");
}

bool SecurityManager::DetectImpersonationAttack (Time currentTime, std::vector<uint32_t> &suspectList)
{
    // Algorithm 4, Lines 11–16:
    // if (V_ID has Multiple different neighbours)
    //     if (multiple different neighbours != Neighbors saved in previous stamp)
    //         suspect [Malicious] = Vehicle ID
    //         suspect [Malicious] Time stamp
    bool detected = false;
    suspectList.clear ();

    for (const auto &pair : m_neighborReports) {
        uint32_t claimedId = pair.first;
        const auto &reports = pair.second;

        if (reports.size () >= 2) {
            // Check if reports for the same claimed ID come from different physical sender nodes or disparate locations
            bool multiLocation = false;
            uint32_t firstRealId = reports[0].realVehicleId;

            for (size_t i = 1; i < reports.size (); ++i) {
                if (reports[i].realVehicleId != firstRealId) {
                    multiLocation = true;
                    break;
                }

                double dx = reports[i].location.x - reports[0].location.x;
                double dy = reports[i].location.y - reports[0].location.y;
                double dist = std::sqrt (dx * dx + dy * dy);

                // Distance threshold > 50m for same timestamp implies multiple physical positions
                if (dist > 50.0) {
                    multiLocation = true;
                    break;
                }
            }

            if (multiLocation) {
                m_suspectList[claimedId] = currentTime;
                suspectList.push_back (claimedId);
                detected = true;

                NS_LOG_WARN ("Algorithm 4: Impersonation Attack Detected! Claimed V_ID " 
                             << claimedId << " marked as Suspect[Malicious] at time " 
                             << currentTime.GetSeconds () << "s");
            }
        }
    }

    return detected;
}

void SecurityManager::IssueIcmpMonitoringAndIsolate (
    uint32_t suspectVehicleId,
    Time currentTime,
    IcmpVanetHeader &icmpHeader)
{
    // Algorithm 4, Lines 17–20:
    // for each RSU in the network do
    //     Promiscuous Change mode
    //     Monitor Suspect [Malicious]
    //     Mark malicious Recent Time stamp
    SetPromiscuousMode (true);

    icmpHeader.SetIcmpType (IcmpVanetHeader::ICMP_ECHO_REQUEST);
    icmpHeader.SetIcmpCode (0);
    icmpHeader.SetTargetVehicleId (suspectVehicleId);
    icmpHeader.SetRsuId (m_rsuId);
    icmpHeader.SetTimestampUs (currentTime.GetMicroSeconds ());

    // Confirm malicious and isolate
    if (!IsMalicious (suspectVehicleId)) {
        m_maliciousList.push_back (suspectVehicleId);
    }

    NS_LOG_WARN ("Algorithm 4 [RSU " << m_rsuId << "]: Promiscuous Mode ENABLED. Monitoring Suspect " 
                                     << suspectVehicleId << ". Issued ICMP Echo Request and isolated ID.");
}

bool SecurityManager::IsMalicious (uint32_t claimedVehicleId) const
{
    for (uint32_t id : m_maliciousList) {
        if (id == claimedVehicleId) {
            return true;
        }
    }
    return false;
}

void SecurityManager::PurgeOldReports (Time currentTime, Time windowSize)
{
    auto it = m_neighborReports.begin ();
    while (it != m_neighborReports.end ()) {
        auto &vec = it->second;
        auto vIt = vec.begin ();
        while (vIt != vec.end ()) {
            if ((currentTime - vIt->timestamp) > windowSize) {
                vIt = vec.erase (vIt);
            } else {
                ++vIt;
            }
        }

        if (vec.empty ()) {
            it = m_neighborReports.erase (it);
        } else {
            ++it;
        }
    }
}

} // namespace ns3
