#ifndef HELLO_MESSAGE_H
#define HELLO_MESSAGE_H

#include <ns3/header.h>
#include <ns3/type-id.h>
#include <ns3/buffer.h>
#include <ns3/vector.h>
#include <ns3/nstime.h>
#include <vector>
#include <cstdint>
#include <cstring>
#include <iostream>

namespace ns3 {

/**
 * @brief HELLO Message Header according to Section 3.2 & Algorithm 1.
 * Broadcasted periodically by vehicles to 1-hop neighbors and RSU.
 * Contains: Vehicle ID, Claimed ID, Lane ID, Road ID, Position (x, y), Instantaneous Speed, Direction Type, Timestamp.
 */
class HelloHeader : public Header {
public:
    HelloHeader();
    ~HelloHeader() override = default;

    static TypeId GetTypeId(void);
    TypeId GetInstanceTypeId(void) const override;
    uint32_t GetSerializedSize(void) const override;
    void Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;
    void Print(std::ostream &os) const override;

    // Getters and Setters
    uint32_t GetVehicleId() const { return m_vehicleId; }
    void SetVehicleId(uint32_t id) { m_vehicleId = id; }

    uint32_t GetClaimedId() const { return m_claimedId; }
    void SetClaimedId(uint32_t id) { m_claimedId = id; }

    uint32_t GetLaneId() const { return m_laneId; }
    void SetLaneId(uint32_t lane) { m_laneId = lane; }

    uint32_t GetRoadId() const { return m_roadId; }
    void SetRoadId(uint32_t road) { m_roadId = road; }

    Vector GetPosition() const { return Vector(m_positionX, m_positionY, 0.0); }
    void SetPosition(const Vector &pos) { m_positionX = pos.x; m_positionY = pos.y; }

    double GetSpeed() const { return m_speed; }
    void SetSpeed(double speed) { m_speed = speed; }

    uint8_t GetDirectionType() const { return m_directionType; }
    void SetDirectionType(uint8_t dir) { m_directionType = dir; }

    uint64_t GetTimestampUs() const { return m_timestampUs; }
    void SetTimestampUs(uint64_t ts) { m_timestampUs = ts; }

private:
    uint32_t m_vehicleId{0};
    uint32_t m_claimedId{0};
    uint32_t m_laneId{0};
    uint32_t m_roadId{0};
    double m_positionX{0.0};
    double m_positionY{0.0};
    double m_speed{0.0};
    uint8_t m_directionType{1};
    uint64_t m_timestampUs{0};

    static uint64_t DoubleToUint64(double val);
    static double Uint64ToDouble(uint64_t u);
};

/**
 * @brief HELLO_NEIGH Message Header according to Section 3.3 & Algorithm 2.
 * Sent by vehicles to RSU containing 1-hop neighbor count and neighbor list.
 */
class HelloNeighHeader : public Header {
public:
    HelloNeighHeader();
    ~HelloNeighHeader() override = default;

    static TypeId GetTypeId(void);
    TypeId GetInstanceTypeId(void) const override;
    uint32_t GetSerializedSize(void) const override;
    void Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;
    void Print(std::ostream &os) const override;

    // Getters and Setters
    uint32_t GetVehicleId() const { return m_vehicleId; }
    void SetVehicleId(uint32_t id) { m_vehicleId = id; }

    uint32_t GetClaimedId() const { return m_claimedId; }
    void SetClaimedId(uint32_t id) { m_claimedId = id; }

    uint32_t GetLaneId() const { return m_laneId; }
    void SetLaneId(uint32_t lane) { m_laneId = lane; }

    Vector GetPosition() const { return Vector(m_positionX, m_positionY, 0.0); }
    void SetPosition(const Vector &pos) { m_positionX = pos.x; m_positionY = pos.y; }

    double GetSpeed() const { return m_speed; }
    void SetSpeed(double speed) { m_speed = speed; }

    uint8_t GetDirectionType() const { return m_directionType; }
    void SetDirectionType(uint8_t dir) { m_directionType = dir; }

    const std::vector<uint32_t>& GetNeighborList() const { return m_neighborList; }
    void SetNeighborList(const std::vector<uint32_t>& list) { m_neighborList = list; }
    void AddNeighbor(uint32_t id) { m_neighborList.push_back(id); }

    uint32_t GetNeighborCount() const { return static_cast<uint32_t>(m_neighborList.size()); }

private:
    uint32_t m_vehicleId{0};
    uint32_t m_claimedId{0};
    uint32_t m_laneId{0};
    double m_positionX{0.0};
    double m_positionY{0.0};
    double m_speed{0.0};
    uint8_t m_directionType{1};
    std::vector<uint32_t> m_neighborList;

    static uint64_t DoubleToUint64(double val);
    static double Uint64ToDouble(uint64_t u);
};

} // namespace ns3

#endif // HELLO_MESSAGE_H
