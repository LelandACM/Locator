/*
 * File: BinaryInterfaceServer.h
 * Created On: 2019-09-14
 * Copyright © 2019 Bosch Rexroth AG
*/

#ifndef BINARYINTERFACESERVER_H
#define BINARYINTERFACESERVER_H

/*
 * C++ Standard lib
 */
#include <string>
#include <cstring>
#include <exception>
#include <list>
#include <mutex>
#include <thread>
#include <atomic>
#include <fstream>
#include <iostream>

/*
 * Linux Netinet header and types
 */
#include <poll.h>
#include <unistd.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>


class BinaryInterfaceServer
{
public:
    /*
     * Constructor: bind immediately. Throws runtime_error on any error
     */
    BinaryInterfaceServer(const uint16_t port):
        Port(port){
        bind();
        listenThreadRun = true;
        listenThreadHandle = std::thread(&BinaryInterfaceServer::listenThread, this);
    }

    /*
     * Destructor: Release OS Handle
     */
    ~BinaryInterfaceServer()
    {
        listenThreadRun = false;
        listenThreadHandle.join();
        close(bindHandle);

            std::lock_guard<std::mutex> guard(listenThreadMutex);
        for (const auto& handle: clientHandles)
        {
            close(handle);
        }
    }

    /*
     * OS - write function
     */
    void write(void* target, int32_t size)
    {
        std::lock_guard<std::mutex> guard(listenThreadMutex);

        for (auto it = clientHandles.begin(); it != clientHandles.end();)
        {
            if (::write(*it, target, size) < 0)
            {
                auto delIt = it;
                ++it;

                close(*delIt);
                clientHandles.erase(delIt);
                std::cout << "Client removed!" << std::endl;
            }
            else
            {
                ++it;
            }
        }
    }

    /*
     * No read function in necessary since a server is write only
     */

    friend void setThreadStatus(BinaryInterfaceServer& server){
        server.listenThreadRun = false ;
    }

private:
    uint16_t Port;
    std::string Address;
    int bindHandle {-1};

    std::list<int> clientHandles;
    std::mutex listenThreadMutex;
    std::thread listenThreadHandle;
    std::atomic<bool> listenThreadRun {false};

    void listenThread()
    {
        while (listenThreadRun)
        {
            //Create Variables to start polling
            pollfd fds;
            fds.fd = bindHandle;
            fds.events = POLLIN | POLLERR | POLLRDHUP;
            fds.revents = 0;

            //Poll for 2 sec
            int32_t rc = poll(&fds, 1, 2000);

            socklen_t Size = 0;
            struct sockaddr_storage newAddress {};
            int newHandle = accept(bindHandle, (sockaddr*) &newAddress, &Size);

            if (newHandle > 0)
            {
                std::lock_guard<std::mutex> guard(listenThreadMutex);
                std::cout << "New client accepted!" << std::endl;
                clientHandles.push_back(newHandle);
            }
        }
    }

    void bind()
    {
        //Initialize return and set values
        int32_t rc = 0;
        int32_t Flag;
        int32_t Error;
        //Ai to ipv4
        int32_t AiFamily = AF_INET;

        bindHandle = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        Error = errno;

        if (bindHandle <= 0)
        {
                throw std::runtime_error("BinaryInterfaceServer::bind: Error creating Stream Socket. Errno: "
                      + std::to_string(Error) + ": " + std::string(strerror(Error)));
        }

        //Activate keepalive
        int optval = 1;
        socklen_t optlen = sizeof(optval);
        rc = setsockopt(bindHandle, SOL_SOCKET, SO_KEEPALIVE, &optval, optlen);
        Error = errno;

        if (rc != 0)
        {
                throw std::runtime_error("BinaryInterfaceServer::bind: Error setting keepalive. Errno: "
                      + std::to_string(Error) + ": " + std::string(strerror(Error)));
        }

        //Activate timeout to 30 secs
        struct timeval timeout;
        timeout.tv_usec = 0;
        timeout.tv_sec  = 30;
        optlen = sizeof(timeout);
        rc = setsockopt(bindHandle, SOL_SOCKET, SO_SNDTIMEO, &timeout, optlen);
        Error = errno;

        if (rc != 0)
        {
            throw std::runtime_error("BinaryInterfaceServer::bind: Error setting Timeout. Errno: "
                      + std::to_string(Error) + ": " + std::string(strerror(Error)));
        }

        //bind socket
        struct sockaddr_in  servaddr4{};        ///< socket V4 address
        servaddr4.sin_family      = AF_INET;
        servaddr4.sin_addr.s_addr = htonl(INADDR_ANY);
        servaddr4.sin_port        = htons(Port);

        rc = ::bind(bindHandle, (const sockaddr*) &servaddr4, sizeof(servaddr4));
        Error = errno;

        if (rc != 0)
        {
            throw std::runtime_error("BinaryInterfaceServer::bind: Error binding port "
                      + std::to_string(Port) + ". Errno: "
                      + std::to_string(Error) + ": " + std::string(strerror(Error)));
        }

        std::cout << "BinareInterfaceServer:: successfully bound port " << Port << std::endl;

        //listen socket
        rc = ::listen(bindHandle, 100);
        Error = errno;

        if (rc != 0)
        {
            throw std::runtime_error("BinaryInterfaceServer::bind: Error listening on port "
                      + std::to_string(Port) + ". Errno: "
                      + std::to_string(Error) + ": " + std::string(strerror(Error)));
        }
    }
};

#endif //BINARYINTERFACESERVER_H
