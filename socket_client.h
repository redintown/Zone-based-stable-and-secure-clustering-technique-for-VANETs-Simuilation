#ifndef SOCKET_CLIENT_H
#define SOCKET_CLIENT_H

#include <string>

class SocketClient
{
public:

    SocketClient();

    bool Connect(const std::string& host, int port);

    std::string Receive();

    bool Send(const std::string& message);

    void Close();

private:

    int m_socket;
};

#endif
