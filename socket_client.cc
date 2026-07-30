#include "socket_client.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <iostream>

SocketClient::SocketClient()
{
    m_socket = -1;
}

bool SocketClient::Connect(const std::string& host, int port)
{
    m_socket = socket(AF_INET, SOCK_STREAM, 0);

    if (m_socket < 0)
    {
        std::cerr << "[SocketClient] Failed to create socket.\n";
        return false;
    }

    sockaddr_in server;
    std::memset(&server, 0, sizeof(server));

    server.sin_family = AF_INET;
    server.sin_port = htons(port);

    if (inet_pton(AF_INET, host.c_str(), &server.sin_addr) <= 0)
    {
        std::cerr << "[SocketClient] Invalid server address.\n";

        close(m_socket);
        m_socket = -1;

        return false;
    }

    if (connect(m_socket,
                reinterpret_cast<sockaddr*>(&server),
                sizeof(server)) < 0)
    {
        std::cerr << "[SocketClient] Connection failed.\n";

        close(m_socket);
        m_socket = -1;

        return false;
    }

    std::cout << "[SocketClient] Connected to "
              << host
              << ":"
              << port
              << std::endl;

    return true;
}

std::string SocketClient::Receive()
{
    if (m_socket < 0)
    {
        return "";
    }

    char buffer[16384];
    std::memset(buffer, 0, sizeof(buffer));

    int bytes = recv(
        m_socket,
        buffer,
        sizeof(buffer) - 1,
        0);

    if (bytes <= 0)
    {
        return "";
    }

    buffer[bytes] = '\0';

    return std::string(buffer);
}

bool SocketClient::Send(const std::string& message)
{
    if (m_socket < 0)
    {
        return false;
    }

    ssize_t sent = send(
        m_socket,
        message.c_str(),
        message.size(),
        0);

    return sent == static_cast<ssize_t>(message.size());
}

void SocketClient::Close()
{
    if (m_socket >= 0)
    {
        close(m_socket);
        m_socket = -1;

        std::cout << "[SocketClient] Connection closed."
                  << std::endl;
    }
}
