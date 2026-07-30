#ifndef NEIGHBOR_MANAGER_H
#define NEIGHBOR_MANAGER_H

#include "hello_message.h"
#include "vehicle.h"
#include <ns3/nstime.h>
#include <ns3/vector.h>
#include <map>
#include <vector>
#include <cstdint>

namespace ns3 {

/**
 * @brief Manages the One-Hop Neighbor Table (OHN) for a vehicle as defined in Algorithm 1.
 * Filters received HELLO messages to ensure neighbors belong to the same lane and direction.
 */
class NeighborManager {
public:
    NeighborManager();
    explicit NeighborManager(uint32_t ownerId);
    ~NeighborManager() = default;

    /**
     * @brief Sets owner vehicle ID.
     */
    void SetOwnerId(uint32_t ownerId) { m_ownerId = ownerId; }

    /**
     * @brief Gets owner vehicle ID.
     */
    uint32_t GetOwnerId() const { return m_ownerId; }

    /**
     * @brief Processes a received HELLO_MSG header according to Algorithm 1 (lines 7–9 & 15–16).
     * If sender is moving in the same direction and same lane, adds/updates sender in OHN table.
     * Otherwise, ignores the sender.
     * 
     * @param header The received HelloHeader
     * @param receiveTime Time at which the HELLO_MSG was received
     * @param myLaneId Lane ID of the receiving vehicle
     * @param myDirectionType Direction type (quadrant 1..4) of the receiving vehicle
     * @return true if neighbor was added/updated; false if ignored
     */
    bool ProcessHelloMessage(const HelloHeader& header, Time receiveTime, uint32_t myLaneId, uint8_t myDirectionType);

    /**
     * @brief Purges stale neighbor entries that have not sent a HELLO_MSG within timeout.
     * 
     * @param currentTime Current simulation time
     * @param timeout Maximum allowed silence duration before entry removal
     * @return Number of removed neighbor entries
     */
    uint32_t PurgeStaleNeighbors(Time currentTime, Time timeout);

    /**
     * @brief Checks if a vehicle is in the OHN table.
     */
    bool HasNeighbor(uint32_t vehicleId) const;

    /**
     * @brief Retrieves a neighbor entry by vehicle ID.
     */
    bool GetNeighbor(uint32_t vehicleId, NeighborEntry& entry) const;

    /**
     * @brief Returns the complete One-Hop Neighbor Table.
     */
    const std::map<uint32_t, NeighborEntry>& GetNeighborTable() const { return m_ohnTable; }

    /**
     * @brief Returns the count of 1-hop neighbors in OHN table.
     */
    uint32_t GetNeighborCount() const { return static_cast<uint32_t>(m_ohnTable.size()); }

    /**
     * @brief Returns a list of all neighbor vehicle IDs.
     */
    std::vector<uint32_t> GetNeighborList() const;

    /**
     * @brief Removes a specific neighbor from the OHN table.
     */
    void RemoveNeighbor(uint32_t vehicleId);

    /**
     * @brief Clears all entries from the OHN table.
     */
    void Clear();

private:
    uint32_t m_ownerId{0};
    std::map<uint32_t, NeighborEntry> m_ohnTable; // One-Hop Neighbor Table (OHN)
};

} // namespace ns3

#endif // NEIGHBOR_MANAGER_H
