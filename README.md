# Zone-based Stable and Secure Clustering Technique for VANETs

![NS-3](https://img.shields.io/badge/Simulator-NS--3.46.1-blue.svg)
![SUMO](https://img.shields.io/badge/Traffic_Simulator-SUMO-orange.svg)
![Python](https://img.shields.io/badge/Python-3.x-yellow.svg)
![C++](https://img.shields.io/badge/C++-17-green.svg)

## 📌 Overview
This repository contains the simulation codebase for a **Zone-based Stable and Secure Clustering Technique in Vehicular Ad-hoc Networks (VANETs)**. The project integrates **NS-3** for network simulation and **SUMO** for realistic traffic mobility generation, synchronized via a Python-based TraCI/TCP bridge.

The proposed hybrid framework utilizes both **Vehicle-to-Vehicle (V2V)** and **Vehicle-to-Infrastructure (V2I)** communications to ensure efficient data dissemination, minimize latency, and detect malicious vehicles performing impersonation attacks.

---

## 🚀 Key Features

* **Dynamic Zone Division:** Divides the RSU coverage area into Active and Dead zones based on the average lane speed and HELLO message intervals.
* **Stable Cluster Head (CH) Selection:** Utilizes a **Relativity Metric (RM)** considering vehicle speed and Euclidean distance to the zone's center to elect the most stable CH.
* **Impersonation Attack Detection:** Employs RSUs in promiscuous mode to detect duplicate vehicle IDs or abnormal location hops using ICMP messages.
* **Hybrid Communication:** Low-latency DSRC for intra-cluster V2V communication and reliable LTE for V2I communication.

---

## 🛠️ Prerequisites
To build and run this simulation, you need the following installed on your system:
* **NS-3** (version 3.46.1)
* **SUMO** (Simulation of Urban MObility)
* **Python 3** (with `traci` library)
* **C++ Compiler** (GCC/G++ supporting C++17)
* **CMake**

---

## 📂 Project Structure

```text
vanet/
├── vanet.cc                  # Main NS-3 simulation script
├── bridge.py                 # Python TCP server for NS-3 & SUMO (TraCI) synchronization
├── CMakeLists.txt            # CMake build configuration
├── Core Managers/
│   ├── zone_manager.cc/.h    # Handles Active/Dead zone calculations
│   ├── rm_manager.cc/.h      # Computes the Relativity Metric (RM)
│   ├── cluster_manager.cc/.h # Executes CH selection (Algorithm 2)
│   ├── maintenance_manager   # Cluster maintenance (Algorithm 3)
│   ├── security_manager.cc   # Impersonation attack detection (Algorithm 4)
├── Network & Nodes/
│   ├── vehicle.h             # Vehicle state and properties
│   ├── hello_message.cc/.h   # HELLO and HELLO_NEIGH packet definitions
│   ├── neighbour_manager.cc  # Manages 1-hop neighbor table (OHN)
│   ├── wifi_manager.cc/.h    # Configures IEEE 802.11p DSRC MAC/PHY layers
│   ├── socket_client.cc/.h   # TCP Client for Python bridge communication
└── statistics_manager.cc/.h  # Collects metrics and generates 'results.csv'

💻 How to Build and Run
Step 1: Build the NS-3 Module
Navigate to the root directory containing your CMakeLists.txt and compile the code:

Bash
mkdir build && cd build
cmake ..
make
Step 2: Start the SUMO-TraCI Bridge
Open a new terminal and run the Python bridge. This will start the TCP server and launch the SUMO GUI.

Bash
python3 vanet/bridge.py
Step 3: Run the Simulation
In your main terminal, execute the compiled NS-3 script:

Bash
./ns3.46.1-zone-secure-vanet-default --simDuration=300 --numVehicles=50
📊 Expected Output & Results
Upon successful completion, the simulation generates a results.csv file containing the following metrics over time:

Time, Vehicle Count, Cluster Count

Packets Sent/Received & PDR (Packet Delivery Rate)

Control Overhead & Average Delay

Cluster Head Changes & Average RM

You can use Python data science libraries (pandas, matplotlib) to visualize this data.
