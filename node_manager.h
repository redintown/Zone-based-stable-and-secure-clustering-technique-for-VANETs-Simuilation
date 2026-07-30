// File: node_manager.h
#ifndef NODE_MANAGER_H
#define NODE_MANAGER_H

#include "vehicle.h"
#include "mobility_manager.h"

#include "ns3/network-module.h"

#include <map>
#include <string>
#include <vector>

class NodeManager
{
public:

    NodeManager();

    void UpdateNodes(const std::vector<ns3::Vehicle>& vehicles);

    ns3::Ptr<ns3::Node> GetNode(const std::string& id);
    std::vector<ns3::Ptr<ns3::Node>> GetAllNodes();

private:

    std::map<std::string, ns3::Ptr<ns3::Node>> m_nodes;

    MobilityManager m_mobilityManager;
};

#endif
