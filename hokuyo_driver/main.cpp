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
#include "BinaryInterfaceServer.h"
#include "BinaryInterfaceStruct.h"

/*
 * File test -> read in binary file and prints the content. Binary file must be
 * formatted without padding and like written by TCP test.
 */
void RunFileTest(const std::string &infilename, const std::string &filename = "")
{
    std::cout << "------------------- Filestream validity test for Binary Pose Interface ---------------------" << std::endl;

    std::ifstream infile(infilename, std::ios::binary);
    if (!infile.is_open())
        return;

    //If Fileout is requested open file in binary io mode
    std::ofstream outfile;
    if (filename != "")
    {
        outfile.open(filename, std::ios::binary);
    }

    //Initialize PoseMessage
    ClientLocalizationPoseDatagram poseMessage;

    try
    {
        while (infile.read(reinterpret_cast<char *>(&poseMessage), sizeof(poseMessage)))
        {
            //Write to file
            if (filename != "")
            {
                try
                {
                    outfile.write(reinterpret_cast<char *>(&poseMessage), sizeof(poseMessage));
                    outfile.flush();
                }
                catch (std::ios_base::failure &ex)
                {
                    throw std::runtime_error("TestClientLocalizationPose::RunFileTest: io-error while writing binary outfile: " + std::string(ex.what()));
                }
            }

            std::cout << "age       = " << poseMessage.age << std::endl;
            std::cout << "timestamp = " << poseMessage.timestamp << std::endl;
            std::cout << "uniqueId  = " << poseMessage.uniqueId << std::endl;
            std::cout << "locState  = " << poseMessage.locState << std::endl;
            std::cout << "x         = " << poseMessage.x << std::endl;
            std::cout << "y         = " << poseMessage.y << std::endl;
            std::cout << "yaw       = " << poseMessage.yaw << std::endl;
            std::cout << "z         = " << poseMessage.z << std::endl;
            std::cout << "qw        = " << poseMessage.qw << std::endl;
            std::cout << "qx        = " << poseMessage.qx << std::endl;
            std::cout << "qy        = " << poseMessage.qy << std::endl;
            std::cout << "qz        = " << poseMessage.qz << std::endl;
            std::cout << "epoch     = " << poseMessage.epoch << std::endl;
            std::cout << "x_odo     = " << poseMessage.x_odo << std::endl;
            std::cout << "y_odo     = " << poseMessage.y_odo << std::endl;
            std::cout << "yaw_odo   = " << poseMessage.yaw_odo << std::endl;
            std::cout << "=====================================" << std::endl;
        }
    }
    catch (std::ios_base::failure &ex)
    {
        throw std::runtime_error("TestClientLocalizationPose::RunFileTest: io-error while reading binary infile: " + std::string(ex.what()));
    }
}

bool containLF(char *data, int rv)
{
    if (data[rv - 1] == 10 && data[rv - 2] == 10)
    {
        return true;
    }
    return false;
}

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
        //Create the client. It will connect to the server while construction
        //initializing all fields.
        BinaryInterfaceClient client("192.168.0.10", 10940);

        //If Fileout is requested open file in binary io mode
        std::ofstream outfile;
        if (filename != "")
        {
            outfile.open(filename, std::ios::binary);
        }

        char buf[24];

        int port = 9090;
        char data[7000];
        int32_t rv = 0;
        char header[3] = "QT";
        header[2] = 10;
        memset(buf, 0, sizeof(buf));
        memset(data, 0, sizeof(data));

        // 发送QT请求
        rv = client.write(&header, sizeof(header));
        rv = client.read(&buf, sizeof(buf));

        if (rv > 0)
        {
            header[0] = 'B';
            header[1] = 'M';
            //发送BM请求
            rv = client.write(&header, sizeof(header));
            rv = client.read(&buf, sizeof(buf));
            //std::cout<<"buf:"<<buf<<"rv:"<<rv<<std::endl;
        }
        try
        {
            BinaryInterfaceServer server(port);
            BinaryInterfaceDummyLaserData laserData(0.025);
            char resultData[7000];
            bool isSart = true;
            memset(resultData, 0, sizeof(resultData));
            while (rv >= 0)
            {
                char gd[13] = "GD0000054000";
                gd[12] = 10;
                //char gd[14] = "MD00000540000";
                //gd[13] = 10;
                if (isSart || gd[0] == 'M')
                {
                    //发送GD请求
                    rv = client.write(&gd, sizeof(gd));
                    isSart = false;
                }

                //接收服务器的请求
                rv = client.read(&data, sizeof(data));
                if (rv < 0)
                {
                    break;
                }
                strncat(resultData, data, rv);
                //std::cout << data;
                //收集到一个完整的数据
                if (containLF(data, rv))
                {
                    //std::cout << "rv: " << rv << " total: "<<strlen(resultData)<< std::endl;
                    isSart = true;
                    //解析resultData
                    //去掉头的数据，头的长度为23
                    char removeHeaderData[strlen(resultData)];
                    char tempData[strlen(resultData)];
                    memset(tempData, 0, sizeof(tempData));
                    memset(removeHeaderData, 0, sizeof(removeHeaderData));
                    int step = 1081;
                    float coord[step];
                    float intensity[step];
                    //是否获取反光数据
                    bool hasIntensity = false;
                    //去掉头的数据，头的长度为23
                    memcpy(removeHeaderData, resultData + 23, sizeof(removeHeaderData)-23);
                    //std::cout<<"removeHeaderData:"<<removeHeaderData<<std::endl ;
                    for (size_t i = 0, index = 0; i < sizeof(removeHeaderData); ++i)
                    {
                        //去掉空格和换行
                        if (removeHeaderData[i] != 10 && i % 66 != 64 && i != strlen(removeHeaderData) - 3)
                        {
                            tempData[index++] = removeHeaderData[i];
                        }
                    }
                    for (int i = 0; i < step; ++i)
                    {
                        if (gd[1] == 'D')
                        {
                            hasIntensity = false;
                            //将十六进制转换成10进制
                            int a = tempData[i * 3] - 48;
                            int A = a << 12;
                            int b = tempData[i * 3 + 1] - 48;
                            int B = b << 6;
                            int c = tempData[i * 3 + 2] - 48;
                            coord[i] = 1.0f * (A + B + c) / 1000.0;
                            std::cout<<"coord:"<<coord[i]<<std::endl;
                        }
                        else
                        {
                            hasIntensity = true;
                            //将十六进制转换成10进制
                            int a = tempData[i * 6] - 48;
                            int A = a << 12;
                            int b = tempData[i * 6 + 1] - 48;
                            int B = b << 6;
                            int c = tempData[i * 6 + 2] - 48;
                            coord[i] = 1.0f * (A + B + c) / 1000.0;
                            int d = tempData[i * 6 + 3] - 48;
                            int D = d << 12;
                            int e = tempData[i * 6 + 4] - 48;
                            int E = e << 6;
                            int f = tempData[i * 6 + 5] - 48;
                            intensity[i] = 1.0f * (D + E + f) / 1000.0;
                        }
                    }
                    //调用服务器数据线程
                    auto sensor = laserData.getWaitConstantScan(coord, hasIntensity, intensity);
                    server.write(&sensor, sizeof(sensor));

                    //置空数据
                    memset(resultData, 0, sizeof(resultData));
                    memset(coord, 0.0f, sizeof(coord));
                }
                memset(data, 0, sizeof(data));
            }
        }
        catch (std::exception &ex)
        {
            std::cerr << "TestClientLocalizationPose died with error!" << std::endl;
            std::cerr << "What: " << std::string(ex.what()) << std::endl;
        }
        std::cout << "Socket call __read__ returned with signal: Socket closed." << std::endl;
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
	//雷达IP
    std::string hostName{""};
    std::string infile{""};
    std::string outfile{""};

    while ((c = getopt(argc, argv, "hi:o:a:")) >= 0)
    {
        switch (c)
        {
        case 'a':
            if (optarg)
            {
                hostName = std::string(optarg);
            }
            break;
        case 'i':
            if (optarg)
            {
                infile = std::string(optarg);
            }
            break;
        case 'o':
            if (optarg)
            {
                outfile = std::string(optarg);
            }
            break;
        case 'h':
            usage = true;
            break;
        default:
            usage = true;
        }
    }

    if (infile != "" && hostName != "")
        usage = true;

    if (usage)
    {
        std::cout << "testinterface [-a <address> ]  [-o <outputfile> ] [-i <inputfile> ] [-h ]" << std::endl;
        std::cout << "     -a/-i address and inputfile cannot be used at the same time!" << std::endl;
        return -1;
    }
    /* =================================
     */

    /* =================================
     * Run file or TCP Test
     */
    try
    {
        if (infile != "")
            RunFileTest(infile, outfile);
        else
        {
            if (hostName == "")
                hostName = "localhost";

            RunTcpTest(hostName, outfile);
        }
    }
    catch (std::exception &ex)
    {
        std::cerr << "TestClientLocalizationPose died with error!" << std::endl;
        std::cerr << "What: " << std::string(ex.what()) << std::endl;
    }
    /* =================================
     */

    return 0;
}
