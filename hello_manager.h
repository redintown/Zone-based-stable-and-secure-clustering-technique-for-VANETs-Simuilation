// File: hello_manager.h
#ifndef HELLO_MANAGER_H
#define HELLO_MANAGER_H

#include "vehicle.h"

#include <map>
#include <string>
#include <vector>

struct HelloInfo
{
    std::string vehicleId;

    int laneId;

    std::string roadId;

    double x;
    double y;

    double speed;

    double angle;

    double timestamp;
};

class HelloManager
{
public:

    HelloManager();

    void Broadcast(std::vector<ns3::Vehicle>& vehicles);

    const std::map<std::string, HelloInfo>&
    GetHelloTable() const;

private:

    std::map<std::string, HelloInfo>
    m_helloTable;
};

#endif
