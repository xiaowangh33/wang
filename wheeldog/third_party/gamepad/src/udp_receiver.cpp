#include "udp_receiver.h"
#include <iostream>
#include <cstring>
#include <cerrno>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>

/**
 * @brief Constructor for UdpReceiver.
 * @param port The port to listen for UDP data.
 */
UdpReceiver::UdpReceiver(int port) : socket_fd_(-1) {
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ == -1) {
        std::cerr << "Failed to create socket." << std::endl;
        return;
    }

    timeval receiveTimeout{};
    receiveTimeout.tv_usec = 100000;
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO,
                   &receiveTimeout, sizeof(receiveTimeout)) < 0) {
        std::cerr << "Failed to configure socket receive timeout." << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return;
    }

    sockaddr_in serverAddress{};
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_addr.s_addr = INADDR_ANY;
    serverAddress.sin_port = htons(port);

    if (bind(socket_fd_, (struct sockaddr*)&serverAddress, sizeof(serverAddress)) < 0) {
        std::cerr << "Failed to bind socket." << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return;
    }
}

/**
 * @brief Receive UDP data.
 * @return A vector containing received data as uint8_t.
 */
std::vector<uint8_t> UdpReceiver::ReceiveData() {
    if (socket_fd_ < 0) {
        return {};
    }

    std::vector<uint8_t> buffer(1024);
    sockaddr_in clientAddress{};
    socklen_t clientAddressLen = sizeof(clientAddress);

    int bytesRead = recvfrom(socket_fd_, buffer.data(), buffer.size(), 0, (struct sockaddr*)&clientAddress, &clientAddressLen);
    if (bytesRead == -1) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
            return {};
        }
        std::cerr << "Failed to receive data." << std::endl;
        return {};
    }

    buffer.resize(bytesRead);

    return buffer;
}

/**
 * @brief Destructor for UdpReceiver.
 */
UdpReceiver::~UdpReceiver() {
    if (socket_fd_ != -1) {
        close(socket_fd_);
    }
}
