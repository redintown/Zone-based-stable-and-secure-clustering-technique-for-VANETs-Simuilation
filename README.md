# Zone-based Stable and Secure Clustering Technique for VANETs

## Overview
This repository contains the NS-3 simulation code for a Zone-based stable and secure clustering technique in Vehicular Ad-hoc Networks (VANETs). The project integrates NS-3 with SUMO (via TraCI/TCP bridge) to simulate realistic traffic mobility and network communication.

## Key Features
* **Stable Clustering:** Uses a Relativity Metric (RM) based on vehicle speed and distance from fixed geographic zones to elect Cluster Heads (CH).
* **Security Management:** Actively detects and isolates impersonation attacks using promiscuous mode monitoring and ICMP messages.
* **Hybrid Communication:** Utilizes both V2V (DSRC) and V2I (LTE) communications to ensure minimal latency and reduced overhead.

## Requirements
* NS-3 (v3.46.1)
* SUMO (Simulation of Urban MObility)
* Python 3 (with `traci` package installed)

## Project Structure
* `vanet.cc`: Main simulation script.
* `bridge.py`: Python TCP server to synchronize NS-3 with SUMO using TraCI.
* `zone_manager.cc` & `cluster_manager.cc`: Handles geographic zones and CH selection.
* `security_manager.cc`: Implements impersonation attack detection (Algorithm 4).

## How to Run

1. **Build the Project**
   Navigate to the project directory and build the NS-3 code:
   ```bash
   mkdir build && cd build
   cmake ..
   make
