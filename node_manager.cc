// File: node_manager.cc
#include "node_manager.h"

#include <iostream>
#include <string>

using namespace ns3;

NodeManager::NodeManager()
{
}

void
NodeManager::UpdateNodes(const std::vector<ns3::Vehicle>& vehicles)
{
    for (const auto& vehicle : vehicles)
    {
        std::string vid = std::to_string(vehicle.GetVehicleId());
        auto it = m_nodes.find(vid);

        if (it == m_nodes.end())
        {
            Ptr<Node> node = CreateObject<Node>();

            m_nodes[vid] = node;

            m_mobilityManager.InitializeNode(vid,
                                             node);

            std::cout
                << "[NS3] Node Created : "
                << vid
                << std::endl;
        }

        m_mobilityManager.UpdatePosition(vehicle);
    }
}

Ptr<Node>
NodeManager::GetNode(const std::string& id)
{
    auto it = m_nodes.find(id);

    if (it != m_nodes.end())
    {
        return it->second;
    }

    return nullptr;
}

std::vector<ns3::Ptr<ns3::Node>>
NodeManager::GetAllNodes()
{
    std::vector<ns3::Ptr<ns3::Node>> nodes;

    for (const auto &entry : m_nodes)
    {
        nodes.push_back(entry.second);
    }

    return nodes;
}
