#include "vehicle.h"
#include "hello_message.h"
#include "zone_manager.h"
#include "neighbour_manager.h"
#include "rm_manager.h"
#include "cluster_manager.h"
#include "maintenance_manager.h"
#include "security_manager.h"
#include "statistics_manager.h"
#include "hello_manager.h"
#include "node_manager.h"
#include "mobility_manager.h"
#include "wifi_manager.h"
#include "socket_client.h"


#include <ns3/core-module.h>
#include <ns3/network-module.h>
#include <ns3/mobility-module.h>
#include <ns3/wifi-module.h>
#include <ns3/internet-module.h>
#include <ns3/applications-module.h>

#include <vector>
#include <map>
#include <iostream>
#include <cmath>
#include <string>
#include <cctype>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("VanetResearchSimulation");

constexpr uint16_t VANET_PORT = 9999;

/**
 * @brief Synchronizes NS-3 vehicle mobility with SUMO via the Python TCP bridge.
 * Reads JSON payload, extracts vehicle data, and updates MobilityModels.
 */
void SyncWithSumo(SocketClient* client,
                  NodeContainer* vehicleNodes,
                  NodeManager* nodeMgr,
                  StatisticsManager* statsMgr) {
    std::string data = client->Receive();
    
    if (data.empty()) {
        Simulator::Schedule(
            Seconds(0.1),
            &SyncWithSumo,
            client,
            vehicleNodes,
            nodeMgr,
            statsMgr);
        return;
    }


    std::vector<Vehicle> parsedVehicles;
    size_t pos = 0;
    
    // Very basic JSON parser to avoid external library dependencies
    while ((pos = data.find("{", pos)) != std::string::npos) {
        size_t endPos = data.find("}", pos);
        if (endPos == std::string::npos) break;
        
        std::string obj = data.substr(pos, endPos - pos);
        
        auto getVal = [&](const std::string& key) -> double {
            size_t k = obj.find("\"" + key + "\"");
            if (k != std::string::npos) {
                size_t c = obj.find(":", k);
                if (c != std::string::npos) {
                    try { return std::stod(obj.substr(c + 1)); } catch(...) {}
                }
            }
            return 0.0;
        };

        auto getStr = [&](const std::string& key) -> std::string {
            size_t k = obj.find("\"" + key + "\"");
            if (k != std::string::npos) {
                size_t c = obj.find(":", k);
                if (c != std::string::npos) {
                    size_t q1 = obj.find("\"", c);
                    size_t q2 = obj.find("\"", q1 + 1);
                    if (q1 != std::string::npos && q2 != std::string::npos) {
                        return obj.substr(q1 + 1, q2 - q1 - 1);
                    }
                    size_t cma = obj.find(",", c);
                    if (cma == std::string::npos) cma = obj.length();
                    std::string val = obj.substr(c + 1, cma - c - 1);
                    val.erase(0, val.find_first_not_of(" \t\r\n"));
                    val.erase(val.find_last_not_of(" \t\r\n") + 1);
                    return val;
                }
            }
            return "";
        };

        std::string vidStr = getStr("vehicleId");
        uint32_t vid = 0;
        for (char ch : vidStr) {
            if (std::isdigit(ch)) {
                vid = vid * 10 + (ch - '0');
            }
        }

        if (vid < vehicleNodes->GetN()) {
            Vehicle v(vid, 0, 0);
            v.SetCurrentPosition(Vector(getVal("x"), getVal("y"), 0.0));
            v.SetSpeed(getVal("speed"));
            parsedVehicles.push_back(v);

            // Directly update the NS-3 Node MobilityModel
            if (vid <= vehicleNodes->GetN()) {
                Ptr<Node> node = vehicleNodes->Get(vid);
                Ptr<MobilityModel> mob = node->GetObject<MobilityModel>();
                if (mob) {
                    mob->SetPosition(Vector(getVal("x"), getVal("y"), 0.0));
                }
            }
        }
        pos = endPos + 1;
    }

    if (!parsedVehicles.empty()) {
        nodeMgr->UpdateNodes(parsedVehicles);

        if (statsMgr)
        {
            std::cout << "[DEBUG] parsedVehicles.size() = "
                      << parsedVehicles.size()
                      << std::endl;
                      
            statsMgr->UpdateVehicleCount(parsedVehicles.size());
        }
    }

    // Acknowledge the bridge to unblock the Python TCP server
    // Note: If SocketClient lacks a Send method, this might require adjusting or removing
    client->Send("ACK\n"); 

    Simulator::Schedule(
        Seconds(0.1),
        &SyncWithSumo,
        client,
        vehicleNodes,
        nodeMgr,
        statsMgr);
    }
    void ExportStatisticsPeriodically(StatisticsManager* statsMgr)
        {
            if (statsMgr)
            {
                statsMgr->UpdateSimulationTime(Simulator::Now().GetSeconds());

                statsMgr->ExportCSV();

                Simulator::Schedule(

                    Seconds(1.0),

                    &ExportStatisticsPeriodically,

                    statsMgr);
            }
        }
/**
 * @brief NS-3 Application running on each Vehicle node.
 * Integrates Vehicle state, HELLO message broadcasting, and neighbor table updates.
 */
class VanetVehicleApp : public Application {
public:
    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("ns3::VanetVehicleApp")
            .SetParent<Application>()
            .SetGroupName("Vanet")
            .AddConstructor<VanetVehicleApp>();
        return tid;
    }

    VanetVehicleApp() = default;
    ~VanetVehicleApp() override = default;

    void Setup(uint32_t vehicleId,
           uint32_t laneId,
           uint32_t roadId,
           Address rsuAddress,
           StatisticsManager* statsMgr) {
        m_vehicle = Vehicle(vehicleId, laneId, roadId);
        m_neighborMgr.SetOwnerId(vehicleId);
        m_rsuAddress = rsuAddress;
        m_statsMgr = statsMgr;
    }

    Vehicle& GetVehicle() { return m_vehicle; }
    NeighborManager& GetNeighborManager() { return m_neighborMgr; }

protected:
    void StartApplication() override {
        m_socket = Socket::CreateSocket(GetNode(), TypeId::LookupByName("ns3::UdpSocketFactory"));
        m_socket->Bind(InetSocketAddress(Ipv4Address::GetAny(), VANET_PORT));
        m_socket->SetAllowBroadcast(true);
        m_socket->SetRecvCallback(MakeCallback(&VanetVehicleApp::ReceivePacket, this));

        // Schedule periodic 1-second HELLO broadcasting (Algorithm 1)
        m_helloEvent = Simulator::Schedule(Seconds(1.0), &VanetVehicleApp::SendHelloMessage, this);
    }

    void StopApplication() override {
        Simulator::Cancel(m_helloEvent);
        if (m_socket) {
            m_socket->Close();
        }
    }

private:
    void SendHelloMessage() {
        // Update mobility info
        Ptr<MobilityModel> mobility = GetNode()->GetObject<MobilityModel>();
        if (mobility) {
            Vector pos = mobility->GetPosition();
            Vector vel = mobility->GetVelocity();
            double speed = std::sqrt(vel.x * vel.x + vel.y * vel.y);
            
            m_vehicle.SetCurrentPosition(pos);
            m_vehicle.SetSpeed(speed);
            
            // Set target position in front along same road vector
            m_vehicle.SetTargetPosition(Vector(pos.x + 1000.0, pos.y, pos.z));
        }

        // Section 3.2 & Algorithm 1: Broadcast HELLO MSG to 1-hop neighbors
        HelloHeader header;
        header.SetVehicleId(m_vehicle.GetVehicleId());
        header.SetClaimedId(m_vehicle.GetClaimedId());
        header.SetLaneId(m_vehicle.GetLaneId());
        header.SetRoadId(m_vehicle.GetRoadId());
        header.SetPosition(m_vehicle.GetCurrentPosition());
        header.SetSpeed(m_vehicle.GetSpeed());
        header.SetDirectionType(m_vehicle.GetDirectionType());
        header.SetTimestampUs(Simulator::Now().GetMicroSeconds());

        Ptr<Packet> packet = Create<Packet>(64); // 64 bytes per Table 2
        packet->AddHeader(header);

        // Broadcast to 1-hop neighbors via DSRC
        m_socket->SendTo(packet, 0, InetSocketAddress(Ipv4Address::GetBroadcast(), VANET_PORT));
        if (m_statsMgr)
        {
            m_statsMgr->RecordHelloPacket();
            m_statsMgr->RecordPacketSent(packet->GetSize());
        }

        // Send HELLO_NEIGH message to RSU (Algorithm 2, Lines 1-3)
        SendHelloNeighMessage();

        // Purge stale neighbors
        m_neighborMgr.PurgeStaleNeighbors(Simulator::Now(), Seconds(2.0));

        // Reschedule next HELLO broadcast
        m_helloEvent = Simulator::Schedule(Seconds(1.0), &VanetVehicleApp::SendHelloMessage, this);
    }

    void SendHelloNeighMessage() {
        HelloNeighHeader neighHeader;
        neighHeader.SetVehicleId(m_vehicle.GetVehicleId());
        neighHeader.SetClaimedId(m_vehicle.GetClaimedId());
        neighHeader.SetLaneId(m_vehicle.GetLaneId());
        neighHeader.SetPosition(m_vehicle.GetCurrentPosition());
        neighHeader.SetSpeed(m_vehicle.GetSpeed());
        neighHeader.SetDirectionType(m_vehicle.GetDirectionType());
        neighHeader.SetNeighborList(m_neighborMgr.GetNeighborList());

        Ptr<Packet> packet = Create<Packet>();
        packet->AddHeader(neighHeader);

        m_socket->SendTo(packet, 0, m_rsuAddress);
        if (m_statsMgr)
        {
            m_statsMgr->RecordHelloNeighborPacket();
            m_statsMgr->RecordPacketSent(packet->GetSize());
        }
    }

    void ReceivePacket(Ptr<Socket> socket) {
        Ptr<Packet> packet;
        Address from;
        while ((packet = socket->RecvFrom(from))) {
            if (m_statsMgr)
            {
                m_statsMgr->RecordPacketReceived(packet->GetSize());
            }
            HelloHeader helloHeader;
            if (packet->PeekHeader(helloHeader) != 0) {
                Ptr<Packet> copy = packet->Copy();
                copy->RemoveHeader(helloHeader);
                if (m_statsMgr)
                {
                    double delay =
                        (Simulator::Now().GetMicroSeconds() -
                         helloHeader.GetTimestampUs()) / 1000000.0;

                    m_statsMgr->RecordDelay(delay);
                }
                
                // Process HELLO message via NeighborManager (Algorithm 1)
                m_neighborMgr.ProcessHelloMessage(
                    helloHeader, 
                    Simulator::Now(), 
                    m_vehicle.GetLaneId(), 
                    m_vehicle.GetDirectionType()
                );
                continue;
            }

            IcmpVanetHeader icmpHeader;
            if (packet->PeekHeader(icmpHeader) != 0) {
                Ptr<Packet> copy = packet->Copy();
                copy->RemoveHeader(icmpHeader);
                if (m_statsMgr)
                {
                    m_statsMgr->RecordICMPPacket();
                }
                                
                // Process Security ICMP Message (Section 3.5 & Algorithm 4)
                if (icmpHeader.GetTargetVehicleId() == m_vehicle.GetClaimedId()) {
                    NS_LOG_INFO("Vehicle " << m_vehicle.GetVehicleId() 
                                           << " received ICMP warning/redirect from RSU " 
                                           << icmpHeader.GetRsuId());
                }
            }
        }
    }

    Vehicle m_vehicle;
    NeighborManager m_neighborMgr;
    Ptr<Socket> m_socket;
    Address m_rsuAddress;
    StatisticsManager* m_statsMgr{nullptr};
    EventId m_helloEvent;
};

NS_OBJECT_ENSURE_REGISTERED(VanetVehicleApp);

/**
 * @brief NS-3 Application running on the Roadside Unit (RSU).
 * Integrates ZoneManager, ClusterManager, MaintenanceManager, and SecurityManager.
 */
class VanetRsuApp : public Application {
public:
    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("ns3::VanetRsuApp")
            .SetParent<Application>()
            .SetGroupName("Vanet")
            .AddConstructor<VanetRsuApp>();
        return tid;
    }

    VanetRsuApp() = default;
    ~VanetRsuApp() override = default;

    void Setup(uint32_t rsuId, 
               Vector pos, 
               ZoneManager* zoneMgr, 
               ClusterManager* clusterMgr, 
               MaintenanceManager* maintMgr, 
               SecurityManager* secMgr,
               StatisticsManager* statsMgr) {
        m_rsuId = rsuId;
        m_position = pos;
        m_zoneMgr = zoneMgr;
        m_clusterMgr = clusterMgr;
        m_maintMgr = maintMgr;
        m_secMgr = secMgr;
        m_statsMgr = statsMgr;
    }

    void RegisterVehicle(Ptr<VanetVehicleApp> vehicleApp) {
        m_vehicleApps[vehicleApp->GetVehicle().GetVehicleId()] = vehicleApp;
    }

protected:
    void StartApplication() override {
        m_socket = Socket::CreateSocket(GetNode(), TypeId::LookupByName("ns3::UdpSocketFactory"));
        m_socket->Bind(InetSocketAddress(Ipv4Address::GetAny(), VANET_PORT));
        m_socket->SetRecvCallback(MakeCallback(&VanetRsuApp::ReceivePacket, this));

        // Schedule periodic CH Selection (Algorithm 2) every 20 seconds
        m_chSelectionEvent = Simulator::Schedule(Seconds(2.0), &VanetRsuApp::PeriodicChSelection, this);

        // Schedule periodic Maintenance (Algorithm 3)
        m_maintEvent = Simulator::Schedule(Seconds(3.0), &VanetRsuApp::PeriodicMaintenanceCheck, this);

        // Schedule periodic Impersonation Attack Detection (Algorithm 4)
        m_securityEvent = Simulator::Schedule(Seconds(4.0), &VanetRsuApp::PeriodicSecurityCheck, this);
    }

    void StopApplication() override {
        Simulator::Cancel(m_chSelectionEvent);
        Simulator::Cancel(m_maintEvent);
        Simulator::Cancel(m_securityEvent);
        if (m_socket) {
            m_socket->Close();
        }
    }

private:
    void ReceivePacket(Ptr<Socket> socket) {
        Ptr<Packet> packet;
        Address from;
        while ((packet = socket->RecvFrom(from))) {
            HelloNeighHeader neighHeader;
            if (packet->PeekHeader(neighHeader) != 0) {
                Ptr<Packet> copy = packet->Copy();
                copy->RemoveHeader(neighHeader);

                // Pass to ClusterManager (Algorithm 2)
                if (m_clusterMgr) {
                    m_clusterMgr->ProcessHelloNeighMessage(neighHeader, Simulator::Now());
                }

                // Track CH activity timestamp for Algorithm 3
                m_chLastHeardMap[neighHeader.GetVehicleId()] = Simulator::Now();

                // Pass neighbor update to SecurityManager (Algorithm 4)
                if (m_secMgr) {
                    uint32_t reportingNodeId = neighHeader.GetVehicleId();
                    m_secMgr->ProcessNeighborUpdate(
                        reportingNodeId,
                        neighHeader.GetClaimedId(),
                        neighHeader.GetVehicleId(),
                        neighHeader.GetPosition(),
                        Simulator::Now()
                    );
                }
            }
        }
    }

    void PeriodicChSelection() {
        if (m_clusterMgr && m_zoneMgr) {
            // Collect updated states from registered vehicle apps
            std::map<uint32_t, Vehicle> vehicleStates;
            for (auto& pair : m_vehicleApps) {
                vehicleStates[pair.first] = pair.second->GetVehicle();
            }

            // Execute Algorithm 2: CH Selection
            auto clusters = m_clusterMgr->RunChSelectionAlgorithm(*m_zoneMgr, vehicleStates);
            if (m_statsMgr)
            {
                m_statsMgr->UpdateClusterCount(static_cast<uint32_t>(clusters.size()));

                uint32_t chCount = 0;

                for (const auto& cluster : clusters)
                {
                    if (cluster.second.chVehicleId != 0)
                    {
                        ++chCount;
                    }
                }

                m_statsMgr->UpdateClusterHeadCount(chCount);
            }
            // Apply updated states back to vehicle apps
            for (auto& pair : vehicleStates) {
                if (m_vehicleApps.find(pair.first) != m_vehicleApps.end()) {
                    m_vehicleApps[pair.first]->GetVehicle().SetMode(pair.second.GetMode());
                    m_vehicleApps[pair.first]->GetVehicle().SetClusterHeadId(pair.second.GetClusterHeadId());
                    m_vehicleApps[pair.first]->GetVehicle().SetRelativityMetric(pair.second.GetRelativityMetric());
                }
            }
        }

        m_chSelectionEvent = Simulator::Schedule(Seconds(20.0), &VanetRsuApp::PeriodicChSelection, this);
    }

    void PeriodicMaintenanceCheck() {
        if (m_maintMgr && m_clusterMgr && m_zoneMgr) {
            std::map<uint32_t, Vehicle> vehicleStates;
            for (auto& pair : m_vehicleApps) {
                vehicleStates[pair.first] = pair.second->GetVehicle();
            }

            // Execute Algorithm 3: Maintenance check
            m_maintMgr->CheckRsuChLinksAndMaintain(
                *m_clusterMgr, 
                *m_zoneMgr, 
                vehicleStates, 
                m_chLastHeardMap, 
                Seconds(5.0), 
                Simulator::Now()
            );
        }

        m_maintEvent = Simulator::Schedule(Seconds(5.0), &VanetRsuApp::PeriodicMaintenanceCheck, this);
    }

    void PeriodicSecurityCheck() {
        if (m_secMgr && m_socket) {
            // Execute Algorithm 4: Impersonation Attack Detection
            std::vector<uint32_t> suspects;
            if (m_secMgr->DetectImpersonationAttack(Simulator::Now(), suspects)) {
                for (uint32_t suspectId : suspects) {
                    IcmpVanetHeader icmpHeader;
                    m_secMgr->IssueIcmpMonitoringAndIsolate(suspectId, Simulator::Now(), icmpHeader);

                    // Broadcast ICMP warning packet
                    Ptr<Packet> packet = Create<Packet>();
                    packet->AddHeader(icmpHeader);
                    m_socket->SendTo(packet, 0, InetSocketAddress(Ipv4Address::GetBroadcast(), VANET_PORT));
                }
            }

            m_secMgr->PurgeOldReports(Simulator::Now(), Seconds(10.0));
        }

        m_securityEvent = Simulator::Schedule(Seconds(2.0), &VanetRsuApp::PeriodicSecurityCheck, this);
    }

    uint32_t m_rsuId{0};
    Vector m_position{0.0, 0.0, 0.0};
    ZoneManager* m_zoneMgr{nullptr};
    ClusterManager* m_clusterMgr{nullptr};
    MaintenanceManager* m_maintMgr{nullptr};
    SecurityManager* m_secMgr{nullptr};
    StatisticsManager* m_statsMgr{nullptr};

    Ptr<Socket> m_socket;
    std::map<uint32_t, Ptr<VanetVehicleApp>> m_vehicleApps;
    std::map<uint32_t, Time> m_chLastHeardMap;

    EventId m_chSelectionEvent;
    EventId m_maintEvent;
    EventId m_securityEvent;
};

NS_OBJECT_ENSURE_REGISTERED(VanetRsuApp);

} // namespace ns3

using namespace ns3;

int main(int argc, char* argv[]) {
    // Simulation Parameters per Table 2
    double simDuration = 300.0;    // 300 s
    uint32_t numVehicles = 50;     // 50 vehicles

    CommandLine cmd(__FILE__);
    cmd.AddValue("simDuration", "Simulation Duration in seconds", simDuration);
    cmd.AddValue("numVehicles", "Number of Vehicles", numVehicles);
    cmd.Parse(argc, argv);

    // Instantiate Plain C++ Managers on Stack
    NodeManager nodeMgr;
    MobilityManager mobMgr;
    HelloManager helloMgr;
    SocketClient socketClient;
    WifiManager wifiMgr;
    ZoneManager zoneMgr;
    RmManager rmMgr(0.5, 0.5);
    ClusterManager clusterMgr(1);
    MaintenanceManager maintMgr(1);
    SecurityManager secMgr(1);
    
    StatisticsManager statsMgr;
    statsMgr.Initialize("results.csv");

    // Connect to Python SUMO Bridge via TCP
    if (!socketClient.Connect("127.0.0.1", 9999)) {
        NS_FATAL_ERROR("Failed to connect to Python bridge on 127.0.0.1:9999. Terminating.");
        return 1;
    }
    NS_LOG_INFO("Successfully connected to SUMO bridge.");

    // Initialize Zone and Manager parameters
    zoneMgr.InitializeRsuZones(1, Vector(2500.0, 20.0, 0.0), 900.0, 3);
    clusterMgr.SetRsuId(1);
    clusterMgr.SetStatisticsManager(&statsMgr);
    clusterMgr.SetRmWeights(0.5, 0.5);
    maintMgr.SetRsuId(1);
    secMgr.SetRsuId(1);

    // Create Nodes
    NodeContainer vehicleNodes;
    vehicleNodes.Create(numVehicles);

    NodeContainer rsuNodes;
    rsuNodes.Create(1);

    // IEEE 802.11p Configuration per Table 2
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss("ns3::TwoRayGroundPropagationLossModel");

    YansWifiPhyHelper wifiPhy;
    wifiPhy.SetChannel(wifiChannel.Create());

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211p);
    
    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");

    NetDeviceContainer vehicleDevices = wifi.Install(wifiPhy, wifiMac, vehicleNodes);
    NetDeviceContainer rsuDevices = wifi.Install(wifiPhy, wifiMac, rsuNodes);

    // Internet Stack setup
    InternetStackHelper internet;
    internet.Install(vehicleNodes);
    internet.Install(rsuNodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer vehicleInterfaces = ipv4.Assign(vehicleDevices);
    Ipv4InterfaceContainer rsuInterfaces = ipv4.Assign(rsuDevices);

    // Mobility Setup
    Ptr<ListPositionAllocator> rsuPositionAlloc = CreateObject<ListPositionAllocator>();
    rsuPositionAlloc->Add(Vector(2500.0, 20.0, 0.0));
    MobilityHelper rsuMobility;
    rsuMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    rsuMobility.SetPositionAllocator(rsuPositionAlloc);
    rsuMobility.Install(rsuNodes);

    MobilityHelper vehicleMobility;
    vehicleMobility.SetPositionAllocator("ns3::RandomBoxPositionAllocator",
        "X", StringValue("ns3::UniformRandomVariable[Min=2000.0|Max=3000.0]"),
        "Y", StringValue("ns3::UniformRandomVariable[Min=10.0|Max=30.0]"),
        "Z", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));

    // Changing to ConstantPositionMobilityModel to allow external SUMO updates without overrides
    vehicleMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    vehicleMobility.Install(vehicleNodes);

    // Install RSU Application
    Ptr<VanetRsuApp> rsuApp = CreateObject<VanetRsuApp>();
    rsuApp->Setup(
        1,
        Vector(2500.0, 20.0, 0.0),
        &zoneMgr,
        &clusterMgr,
        &maintMgr,
        &secMgr,
        &statsMgr);
    rsuNodes.Get(0)->AddApplication(rsuApp);
    rsuApp->SetStartTime(Seconds(0.0));
    rsuApp->SetStopTime(Seconds(simDuration));

    // Install Vehicle Applications
    Address rsuAddress = InetSocketAddress(rsuInterfaces.GetAddress(0), VANET_PORT);

    for (uint32_t i = 0; i < numVehicles; ++i) {
        uint32_t laneId = (i % 3) + 1;
        Ptr<VanetVehicleApp> vehicleApp = CreateObject<VanetVehicleApp>();
        uint32_t vehicleId = i + 1;

        vehicleApp->Setup(
            vehicleId,
            laneId,
            1,
            rsuAddress,
            &statsMgr);
        vehicleApp->GetVehicle().SetIpAddress(vehicleInterfaces.GetAddress(i));
        
        vehicleNodes.Get(i)->AddApplication(vehicleApp);
        vehicleApp->SetStartTime(Seconds(0.1 * i));
        vehicleApp->SetStopTime(Seconds(simDuration));

        rsuApp->RegisterVehicle(vehicleApp);

        // Inject simulated impersonation attack on Node 31 changing ID to 4 (Section 4.2.5)
        if (vehicleId == 31) {
            Simulator::Schedule(Seconds(15.0), [vehicleApp]() {
                vehicleApp->GetVehicle().SetClaimedId(4);
                NS_LOG_WARN("Simulated Attack: Vehicle 31 changed claimed ID to 4");
            });
        }
    }

    NS_LOG_INFO("Starting NS-3.46 VANET Simulation: " << numVehicles << " Vehicles, Duration: " << simDuration << "s");

    // Schedule the continuous sync events starting at time 0
    Simulator::Schedule(Seconds(0.0), &SyncWithSumo, &socketClient, &vehicleNodes, &nodeMgr, &statsMgr);
    Simulator::Schedule(
        Seconds(1.0),
        &ExportStatisticsPeriodically,
        &statsMgr);
        
    Simulator::Stop(Seconds(simDuration));
    Simulator::Run();

    statsMgr.PrintSummary();

    Simulator::Destroy();

    socketClient.Close();
    NS_LOG_INFO("Simulation Completed Successfully. Socket Closed.");
    return 0;
}
