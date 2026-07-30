#include "neighbour_manager.h"
#include <ns3/log.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("NeighbourManager");

NeighborManager::NeighborManager ()
    : m_ownerId (0)
{
}

NeighborManager::NeighborManager (uint32_t ownerId)
    : m_ownerId (ownerId)
{
}

bool NeighborManager::ProcessHelloMessage (const HelloHeader &header, Time receiveTime, uint32_t myLaneId, uint8_t myDirectionType)
{
    uint32_t senderId = header.GetVehicleId ();
    
    // Do not add self to neighbor table
    if (senderId == m_ownerId) {
        return false;
    }

    uint32_t senderLaneId = header.GetLaneId ();
    uint8_t senderDirectionType = header.GetDirectionType ();

    // Algorithm 1, Line 8: "if m is moving in the same direction and same lane then"
    if (senderLaneId == myLaneId && senderDirectionType == myDirectionType) {
        // Algorithm 1, Line 9: "add m to its neighbouring table OHN"
        NeighborEntry entry (
            senderId,
            senderLaneId,
            header.GetRoadId (),
            header.GetPosition (),
            header.GetSpeed (),
            senderDirectionType,
            receiveTime
        );

        m_ohnTable[senderId] = entry;
        NS_LOG_INFO ("Owner " << m_ownerId << " added/updated neighbor " << senderId 
                               << " in lane " << senderLaneId << " dir " << static_cast<uint32_t> (senderDirectionType));
        return true;
    } else {
        // Algorithm 1, Line 15-16: "Else do nothing"
        NS_LOG_LOGIC ("Owner " << m_ownerId << " ignored HELLO from sender " << senderId 
                               << ": lane mismatch (" << senderLaneId << " vs " << myLaneId 
                               << ") or dir mismatch (" << static_cast<uint32_t> (senderDirectionType) 
                               << " vs " << static_cast<uint32_t> (myDirectionType) << ")");
        return false;
    }
}

uint32_t NeighborManager::PurgeStaleNeighbors (Time currentTime, Time timeout)
{
    uint32_t removedCount = 0;
    auto it = m_ohnTable.begin ();
    while (it != m_ohnTable.end ()) {
        if ((currentTime - it->second.lastTimestamp) > timeout) {
            NS_LOG_INFO ("Owner " << m_ownerId << " purging stale neighbor " << it->first);
            it = m_ohnTable.erase (it);
            removedCount++;
        } else {
            ++it;
        }
    }
    return removedCount;
}

bool NeighborManager::HasNeighbor (uint32_t vehicleId) const
{
    return m_ohnTable.find (vehicleId) != m_ohnTable.end ();
}

bool NeighborManager::GetNeighbor (uint32_t vehicleId, NeighborEntry &entry) const
{
    auto it = m_ohnTable.find (vehicleId);
    if (it != m_ohnTable.end ()) {
        entry = it->second;
        return true;
    }
    return false;
}

std::vector<uint32_t> NeighborManager::GetNeighborList () const
{
    std::vector<uint32_t> list;
    list.reserve (m_ohnTable.size ());
    for (const auto &pair : m_ohnTable) {
        list.push_back (pair.first);
    }
    return list;
}

void NeighborManager::RemoveNeighbor (uint32_t vehicleId)
{
    m_ohnTable.erase (vehicleId);
}

void NeighborManager::Clear ()
{
    m_ohnTable.clear ();
}

} // namespace ns3
