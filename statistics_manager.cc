// File: statistics_manager.cc
#include "statistics_manager.h"

#include <fstream>
#include <iomanip>
#include <iostream>

namespace ns3 {

StatisticsManager::StatisticsManager() {
    Reset();
}

StatisticsManager::~StatisticsManager() {
    if (m_csvFile.is_open()) {
        m_csvFile.close();
    }
}

void StatisticsManager::Initialize(const std::string& csvFilename) {
    m_csvFilename = csvFilename;

    bool writeHeader = false;

    std::ifstream in(csvFilename);
    if (!in.good() || in.peek() == std::ifstream::traits_type::eof()) {
        writeHeader = true;
    }
    in.close();

    m_csvFile.open(csvFilename, std::ios::out | std::ios::app);

    if (writeHeader && m_csvFile.is_open()) {
        m_csvFile
            << "Time,"
            << "VehicleCount,"
            << "ClusterCount,"
            << "PacketsSent,"
            << "PacketsReceived,"
            << "PDR,"
            << "AverageDelay,"
            << "Throughput,"
            << "HelloPackets,"
            << "HelloNeighborPackets,"
            << "ICMPPackets,"
            << "ControlOverhead,"
            << "AverageRM,"
            << "ClusterHeadChanges\n";

        m_csvFile.flush();
    }
}

void StatisticsManager::Reset() {
    m_simTime = 0.0;

    m_vehicleCount = 0;
    m_clusterCount = 0;
    m_clusterHeads = 0;
    m_chChanges = 0;

    m_totalClusterLifetime = 0.0;
    m_destroyedClusters = 0;

    m_packetsSent = 0;
    m_packetsReceived = 0;
    m_totalBytesReceived = 0;

    m_totalDelay = 0.0;
    m_delaySamples = 0;

    m_helloPackets = 0;
    m_helloNeighborPackets = 0;
    m_icmpPackets = 0;

    m_totalRm = 0.0;
    m_rmSamples = 0;
}

void StatisticsManager::RecordPacketSent(uint32_t) {
    ++m_packetsSent;
}

void StatisticsManager::RecordPacketReceived(uint32_t bytes) {
    ++m_packetsReceived;
    m_totalBytesReceived += bytes;
}

void StatisticsManager::RecordDelay(double delaySeconds) {
    m_totalDelay += delaySeconds;
    ++m_delaySamples;
}

void StatisticsManager::RecordClusterCreated() {
    ++m_clusterHeads;
}

void StatisticsManager::RecordClusterDestroyed(double lifetimeSeconds) {
    m_totalClusterLifetime += lifetimeSeconds;
    ++m_destroyedClusters;
}

void StatisticsManager::RecordClusterHeadChange() {
    ++m_chChanges;
}

void StatisticsManager::RecordHelloPacket() {
    ++m_helloPackets;
}

void StatisticsManager::RecordHelloNeighborPacket() {
    ++m_helloNeighborPackets;
}

void StatisticsManager::RecordICMPPacket() {
    ++m_icmpPackets;
}

void StatisticsManager::RecordRM(double rm) {
    m_totalRm += rm;
    ++m_rmSamples;
}

void StatisticsManager::UpdateVehicleCount(uint32_t count) {
    if (count > m_vehicleCount)
    {
        m_vehicleCount = count;
    }
}

void StatisticsManager::UpdateClusterCount(uint32_t count) {
    m_clusterCount = count;
}

void StatisticsManager::UpdateClusterHeadCount(uint32_t count)
{
    m_clusterHeads = count;
}

void StatisticsManager::UpdateSimulationTime(double timeSeconds) {
    m_simTime = timeSeconds;
}

double StatisticsManager::GetAverageClusterLifetime() const {
    if (m_destroyedClusters == 0)
        return 0.0;

    return m_totalClusterLifetime / m_destroyedClusters;
}

double StatisticsManager::GetPDR() const {
    if (m_packetsSent == 0)
        return 0.0;

    return static_cast<double>(m_packetsReceived) /
           static_cast<double>(m_packetsSent);
}

double StatisticsManager::GetAverageDelay() const {
    if (m_delaySamples == 0)
        return 0.0;

    return m_totalDelay / m_delaySamples;
}

double StatisticsManager::GetThroughput() const {
    if (m_simTime <= 0.0)
        return 0.0;

    return (m_totalBytesReceived * 8.0) / m_simTime;
}

uint32_t StatisticsManager::GetControlOverhead() const {
    return m_helloPackets +
           m_helloNeighborPackets +
           m_icmpPackets;
}

double StatisticsManager::GetAverageRM() const {
    if (m_rmSamples == 0)
        return 0.0;

    return m_totalRm / m_rmSamples;
}

void StatisticsManager::ExportCSV() {
    if (!m_csvFile.is_open())
        return;

    m_csvFile << std::fixed << std::setprecision(6)
              << m_simTime << ","
              << m_vehicleCount << ","
              << m_clusterCount << ","
              << m_packetsSent << ","
              << m_packetsReceived << ","
              << GetPDR() << ","
              << GetAverageDelay() << ","
              << GetThroughput() << ","
              << m_helloPackets << ","
              << m_helloNeighborPackets << ","
              << m_icmpPackets << ","
              << GetControlOverhead() << ","
              << GetAverageRM() << ","
              << m_chChanges
              << "\n";

    m_csvFile.flush();
}

void StatisticsManager::PrintSummary() const {
    std::cout << "\n================ Simulation Summary ================\n";
    std::cout << "Simulation Time      : " << m_simTime << " s\n";
    std::cout << "Vehicles             : " << m_vehicleCount << "\n";
    std::cout << "Clusters             : " << m_clusterCount << "\n";
    std::cout << "Packets Sent         : " << m_packetsSent << "\n";
    std::cout << "Packets Received     : " << m_packetsReceived << "\n";
    std::cout << "Packet Delivery Rate : " << GetPDR() * 100.0 << " %\n";
    std::cout << "Average Delay        : " << GetAverageDelay() << " s\n";
    std::cout << "Throughput           : " << GetThroughput() << " bps\n";
    std::cout << "HELLO Packets        : " << m_helloPackets << "\n";
    std::cout << "HELLO_NEIGH Packets  : " << m_helloNeighborPackets << "\n";
    std::cout << "ICMP Packets         : " << m_icmpPackets << "\n";
    std::cout << "Control Overhead     : " << GetControlOverhead() << "\n";
    std::cout << "Average RM           : " << GetAverageRM() << "\n";
    std::cout << "CH Changes           : " << m_chChanges << "\n";
    std::cout << "====================================================\n";
}

} // namespace ns3
