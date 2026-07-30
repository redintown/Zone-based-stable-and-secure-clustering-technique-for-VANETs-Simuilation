// File: statistics_manager.h
#ifndef STATISTICS_MANAGER_H
#define STATISTICS_MANAGER_H

#include <string>
#include <fstream>
#include <cstdint>

namespace ns3 {

class StatisticsManager {
public:
    StatisticsManager();
    ~StatisticsManager();

    void Initialize(const std::string& csvFilename = "results.csv");
    void Reset();

    void RecordPacketSent(uint32_t bytes = 0);
    void RecordPacketReceived(uint32_t bytes = 0);
    void RecordDelay(double delaySeconds);

    void RecordClusterCreated();
    void RecordClusterDestroyed(double lifetimeSeconds = 0.0);
    void RecordClusterHeadChange();

    void RecordHelloPacket();
    void RecordHelloNeighborPacket();
    void RecordICMPPacket();

    void RecordRM(double rm);

    void UpdateVehicleCount(uint32_t count);
    void UpdateClusterCount(uint32_t count);
    void UpdateClusterHeadCount(uint32_t count);
    void UpdateSimulationTime(double timeSeconds);

    void ExportCSV();
    void PrintSummary() const;

    double GetAverageClusterLifetime() const;
    double GetPDR() const;
    double GetAverageDelay() const;
    double GetThroughput() const;
    uint32_t GetControlOverhead() const;
    double GetAverageRM() const;

private:
    double m_simTime;

    uint32_t m_vehicleCount;
    uint32_t m_clusterCount;
    uint32_t m_clusterHeads;
    uint32_t m_chChanges;

    double m_totalClusterLifetime;
    uint32_t m_destroyedClusters;

    uint32_t m_packetsSent;
    uint32_t m_packetsReceived;
    uint64_t m_totalBytesReceived;

    double m_totalDelay;
    uint32_t m_delaySamples;

    uint32_t m_helloPackets;
    uint32_t m_helloNeighborPackets;
    uint32_t m_icmpPackets;

    double m_totalRm;
    uint32_t m_rmSamples;

    std::string m_csvFilename;
    std::ofstream m_csvFile;
};

} // namespace ns3

#endif
