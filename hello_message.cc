#include "hello_message.h"
#include <ns3/log.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("HelloMessage");

uint64_t HelloHeader::DoubleToUint64 (double val)
{
    uint64_t u;
    std::memcpy (&u, &val, sizeof (double));
    return u;
}

double HelloHeader::Uint64ToDouble (uint64_t u)
{
    double val;
    std::memcpy (&val, &u, sizeof (double));
    return val;
}

HelloHeader::HelloHeader ()
    : m_vehicleId (0),
      m_claimedId (0),
      m_laneId (0),
      m_roadId (0),
      m_positionX (0.0),
      m_positionY (0.0),
      m_speed (0.0),
      m_directionType (1),
      m_timestampUs (0)
{
}

TypeId HelloHeader::GetTypeId (void)
{
    static TypeId tid = TypeId ("ns3::HelloHeader")
        .SetParent<Header> ()
        .SetGroupName ("Vanet")
        .AddConstructor<HelloHeader> ();
    return tid;
}

TypeId HelloHeader::GetInstanceTypeId (void) const
{
    return GetTypeId ();
}

uint32_t HelloHeader::GetSerializedSize (void) const
{
    return 49;
}

void HelloHeader::Serialize (Buffer::Iterator start) const
{
    start.WriteHtonU32 (m_vehicleId);
    start.WriteHtonU32 (m_claimedId);
    start.WriteHtonU32 (m_laneId);
    start.WriteHtonU32 (m_roadId);
    start.WriteHtonU64 (DoubleToUint64 (m_positionX));
    start.WriteHtonU64 (DoubleToUint64 (m_positionY));
    start.WriteHtonU64 (DoubleToUint64 (m_speed));
    start.WriteU8 (m_directionType);
    start.WriteHtonU64 (m_timestampUs);
}

uint32_t HelloHeader::Deserialize (Buffer::Iterator start)
{
    m_vehicleId = start.ReadNtohU32 ();
    m_claimedId = start.ReadNtohU32 ();
    m_laneId = start.ReadNtohU32 ();
    m_roadId = start.ReadNtohU32 ();
    m_positionX = Uint64ToDouble (start.ReadNtohU64 ());
    m_positionY = Uint64ToDouble (start.ReadNtohU64 ());
    m_speed = Uint64ToDouble (start.ReadNtohU64 ());
    m_directionType = start.ReadU8 ();
    m_timestampUs = start.ReadNtohU64 ();
    return GetSerializedSize ();
}

void HelloHeader::Print (std::ostream &os) const
{
    os << "VehicleId=" << m_vehicleId
       << " ClaimedId=" << m_claimedId
       << " LaneId=" << m_laneId
       << " RoadId=" << m_roadId
       << " Pos=(" << m_positionX << "," << m_positionY << ")"
       << " Speed=" << m_speed
       << " DirType=" << static_cast<uint32_t> (m_directionType)
       << " TimestampUs=" << m_timestampUs;
}

// ------------------------------------------------------------------
// HelloNeighHeader Implementation
// ------------------------------------------------------------------

uint64_t HelloNeighHeader::DoubleToUint64 (double val)
{
    uint64_t u;
    std::memcpy (&u, &val, sizeof (double));
    return u;
}

double HelloNeighHeader::Uint64ToDouble (uint64_t u)
{
    double val;
    std::memcpy (&val, &u, sizeof (double));
    return val;
}

HelloNeighHeader::HelloNeighHeader ()
    : m_vehicleId (0),
      m_claimedId (0),
      m_laneId (0),
      m_positionX (0.0),
      m_positionY (0.0),
      m_speed (0.0),
      m_directionType (1)
{
}

TypeId HelloNeighHeader::GetTypeId (void)
{
    static TypeId tid = TypeId ("ns3::HelloNeighHeader")
        .SetParent<Header> ()
        .SetGroupName ("Vanet")
        .AddConstructor<HelloNeighHeader> ();
    return tid;
}

TypeId HelloNeighHeader::GetInstanceTypeId (void) const
{
    return GetTypeId ();
}

uint32_t HelloNeighHeader::GetSerializedSize (void) const
{
    return 41 + 4 * static_cast<uint32_t> (m_neighborList.size ());
}

void HelloNeighHeader::Serialize (Buffer::Iterator start) const
{
    start.WriteHtonU32 (m_vehicleId);
    start.WriteHtonU32 (m_claimedId);
    start.WriteHtonU32 (m_laneId);
    start.WriteHtonU64 (DoubleToUint64 (m_positionX));
    start.WriteHtonU64 (DoubleToUint64 (m_positionY));
    start.WriteHtonU64 (DoubleToUint64 (m_speed));
    start.WriteU8 (m_directionType);
    
    uint32_t count = static_cast<uint32_t> (m_neighborList.size ());
    start.WriteHtonU32 (count);
    for (uint32_t id : m_neighborList) {
        start.WriteHtonU32 (id);
    }
}

uint32_t HelloNeighHeader::Deserialize (Buffer::Iterator start)
{
    m_vehicleId = start.ReadNtohU32 ();
    m_claimedId = start.ReadNtohU32 ();
    m_laneId = start.ReadNtohU32 ();
    m_positionX = Uint64ToDouble (start.ReadNtohU64 ());
    m_positionY = Uint64ToDouble (start.ReadNtohU64 ());
    m_speed = Uint64ToDouble (start.ReadNtohU64 ());
    m_directionType = start.ReadU8 ();
    
    uint32_t count = start.ReadNtohU32 ();
    m_neighborList.clear ();
    for (uint32_t i = 0; i < count; ++i) {
        m_neighborList.push_back (start.ReadNtohU32 ());
    }
    return GetSerializedSize ();
}

void HelloNeighHeader::Print (std::ostream &os) const
{
    os << "VehicleId=" << m_vehicleId
       << " ClaimedId=" << m_claimedId
       << " LaneId=" << m_laneId
       << " Pos=(" << m_positionX << "," << m_positionY << ")"
       << " Speed=" << m_speed
       << " DirType=" << static_cast<uint32_t> (m_directionType)
       << " NeighborCount=" << m_neighborList.size ();
}

} // namespace ns3
