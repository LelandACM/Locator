#include <iostream>
#include <string>

/*
 * Parser for command line parameters
 */
#include <getopt.h>

/*
 * C++ Standard Library headers
 */
#include "BinaryInterfaceDummyLaserData.h"
#include "BinaryInterfaceServer.h"
#include "pavo_driver.h"
#include <exception>
/*
 * Run Tcp Test -> This accepts an hostname and (optionally) a filename
 * Connect to the TCP Port 9011 (default port of pose interface)
 * if a filename is set,
 *  -> this will be an output file, where the binary contenct from the socket is written to
 */
void RunTcpTest(const std::string &hostName, const std::string &filename = "")
{
    std::cout << "------------------- Client test for unsecured (no-TLS) Binary Pose Interface ---------------------\n";
    try
    {
        while (true)
        {
            pavo::pavo_driver *driver = new pavo::pavo_driver("192.168.0.11", 2368);
            bool isRunning = driver->pavo_open(hostName, 2368);
            if (!isRunning)
            {
                std::cout << "开启失败！\n";
                return;
            }
            int port = 9090;
            BinaryInterfaceDummyLaserData laserData(0.2);
            BinaryInterfaceServer server(port);
            isRunning = driver->is_lidar_connected();
            driver->enable_motor(true);
            driver->set_motor_speed(25);
            driver->set_merge_coef(1);
            while (true)
            {
                try
                {
                    std::vector<pavo_response_scan_t> vec;
                    bool isOK = driver->get_scanned_data(vec, 0);
                    if (!isOK)
                    {
                        std::cout << "获取数据失败！\n";
                    }
                    int i = 0;
                    std::cout << "size = " << vec.size() << std::endl;
                    float* coord = new float[vec.size()];
                    for (pavo_response_scan_t pcd : vec)
                    {
                        coord[i] = pcd.distance * 0.002;
                        i++;
                    }
                    //调用服务器数据线程
                    auto sensor = laserData.getWaitConstantScan(coord);
                    server.write(&sensor, sizeof(sensor));
                }
                catch (std::exception &ex)
                {
                    std::cout << "connect break\n";
                    driver->pavo_close();
                    sleep(1);
                    continue;
                }
            }
        }
    }
    catch (std::exception &ex)
    {
        std::cerr << "TestClientLocalizationPose died with error!" << std::endl;
        std::cerr << "What: " << std::string(ex.what()) << std::endl;
    }
    std::cout << "Socket call __read__ returned with signal: Socket closed." << std::endl;
}

int main(int argc, char **argv)
{
    /* =================================
     * Command line parsing8
     */
    int c;
    bool usage{false};
    std::string hostName{"192.168.0.12"};
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
        if (hostName == "")
            hostName = "localhost";
        RunTcpTest(hostName, outfile);
    }
    catch (std::exception &ex)
    {
        std::cerr << "TestClientLocalizationPose died with error!" << std::endl;
        std::cerr << "What: " << std::string(ex.what()) << std::endl;
    }
    return 0;
}
