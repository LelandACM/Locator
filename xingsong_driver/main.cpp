/*
 * File: TestClientLocalizationPose.cpp
 * Created On: 2019-03-13
 * Copyright © 2019 Bosch Rexroth AG
*/

/*
 * Parser for command line parameters
 */
#include <cstddef>
#include <getopt.h>

/*
 * C++ Standard Library headers
 */
#include <assert.h>
#include <exception>
#include <fstream>
#include <iostream>
#include <signal.h>
#include <string>

/*
 * MessageStructs and BinaryClient Objects
 */
#include "BinaryInterfaceClient.h"
#include "BinaryInterfaceDummyLaserData.h"
#include "BinaryInterfacePoseStruct.h"
#include "BinaryInterfaceServer.h"
#include "BinaryInterfaceStruct.h"
#include "hins/xingsong_driver.h"

struct PLC_POSE
{
    int state;
    int x;
    int y;
    int yaw;
    PLC_POSE(int _state, int _x, int _y, int _yaw) : state(_state), x(_x), y(_y), yaw(_yaw) {}
};

/*
 * Run Tcp Test -> This accepts an hostname and (optionally) a filename
 * Connect to the TCP Port 9011 (default port of pose interface)
 * if a filename is set,
 *  -> this will be an output file, where the binary contenct from the socket is written to
 */
void RunTcpTest(const std::string &hostName, const std::string &filename = "")
{
    std::cout << "------------------- Client test for unsecured (no-TLS) Binary Pose Interface ---------------------" << std::endl;
    try
    {
        string server_address = "192.168.1.88";
        int port = 8080;
        //1. 创建雷达驱动
        hins::ConnectionAddress laser_conn_info(server_address, port);
        hins::XingSongDriverHdr driver_hdr = std::make_shared<hins::XingSongDriver>(laser_conn_info);
        try
        {
            //2.创建数据服务器转发到力士乐定位软件
            BinaryInterfaceServer server(9090);
            //3.DummyLaserData数据对象
            BinaryInterfaceDummyLaserData laserData(0.0333);
            unsigned int length = 1;
            while (length >= 0)
            {
                //4.获取雷达数据
                hins::ScanData data = driver_hdr->GetFullScan();
                length = data.distance_data.size();
                float *ranges = new float[length];
                float *indensities = new float[length];
                std::cout << "size = " << length << std::endl;
                for (std::size_t i = 0; i < length; i++)
                {
                    ranges[i] = float(data.distance_data[i]) / 1000.0f;
                    //反光数据
                    indensities[i] = data.amplitude_data[i];
                }
                if (length > 0 && length < 4000)
                {
                    //5.将数据构建成BinaryInterfaceDummyLaserData类
                    auto sensor = laserData.getWaitConstantScan(ranges, false, indensities, length);
                    server.write(&sensor, sizeof(sensor));
                }
            }
            std::cout << "Socket call __read__ returned with signal: Socket closed." << std::endl;
        }
        catch (std::exception &ex)
        {
            std::cerr << "TestClientLocalizationPose died with error!" << std::endl;
            std::cerr << "What: " << std::string(ex.what()) << std::endl;
        }
    }
    catch (std::exception &ex)
    {
        std::cerr << "Error during TCP Receiving test: " << std::string(ex.what()) << std::endl;
    }
}

int main(int argc, char **argv)
{
    //Ignore sigpipe
    signal(SIGPIPE, SIG_IGN);

    /* =================================
     * Command line parsing
     */
    int c;
    bool server{true};
    bool usage{false};
    std::string hostName{""};
    std::string outfile{""};

    /* =================================
     * Run file or TCP Test
     */
    try
    {
        if (hostName == "")
            hostName = "localhost";

        RunTcpTest(hostName, outfile);
    }
    catch (std::exception &ex)
    {
        std::cerr << "TestClientHison died with error!" << std::endl;
        std::cerr << "What: " << std::string(ex.what()) << std::endl;
    }
    return 0;
}