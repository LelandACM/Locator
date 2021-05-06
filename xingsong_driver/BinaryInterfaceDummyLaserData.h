/*
 * File: BinaryInterfaceDummyLaserData.h
 * Created On: 2019-09-14
 * Copyright © 2019 Bosch Rexroth AG
*/

#ifndef BINARYINTERFACEDUMMYLASERSCAN_H
#define BINARYINTERFACEDUMMYLASERSCAN_H

#include <chrono>
#include <iostream>
#include <math.h>
#include <thread>

/*
 * Constants of this DummyLaserData provider
 */
constexpr size_t nBeams = 3331;  //根据观测，兴颂的默认数据长度为3330

/*
 * Dummy laser data -> class allows to get dummy (constant range) but valid
 * data as expected by the laser interface.
 *
 * Defines the ClientSensorLaserDatagram for the special case of constant number of beams.
 */
class BinaryInterfaceDummyLaserData
{
public:
    /*
     * structure containing the exact message from the ClientLocalizationPose Interface
     */
    struct __attribute__((packed)) ClientSensorLaserDatagram
    {
        uint16_t scanNum{0};                             //Overwrite this before send, increase every time.
        double time_start{0.};                           //Overwrite this before send, set to current time.
        uint64_t uniqueId{0};                            //This feature is not used and therefore no uniqueId must be provided
        double duration_beam{0.};                        //Overwrite this before send, set constant (see constructor of BinaryInterfaceDummyLaserData)
        double duration_scan{0.};                        //Overwrite this before send, set constant (see constructor of BinaryInterfaceDummyLaserData)
        double duration_rotate{0.};                      //Overwrite this before send, set constant (see constructor of BinaryInterfaceDummyLaserData)
        uint32_t numBeams{nBeams};                       //Constant
        float angleStart{-M_PI * 135/180};                           //Constant: laser opening angle is: -pi/2 to pi/2
        float angleEnd{M_PI * 135/180};                        //Constant: laser opening angle is: -pi/2 to pi/2
        float angleInc{M_PI / 180 * 270 / (nBeams-1)};   //Constant: laser opening angle is: -pi/2 to pi/2
        float minRange{0.};                              //Constant: no ranges smaller than 0
        float maxRange{0.};                              //Constant: no ranges bigger than 60
        uint32_t rangeArraySize{(nBeams)};               //Constant
        float ranges[nBeams];                            //Overwrite this before send, set constant to range
        bool hasIntensities{false};                      //Constant: false
        float minIntensities{0.};                        //Constant: 0
        float maxIntensities{0.};                        //Constant: 0
        uint32_t intensitiesArraySize{nBeams};           //Constant: 0
        float intensities[nBeams];                       // Array of beam reﬂection intensities, as measured by the scanner. Each element corresponding to one beam.
    };

    /*
     * Constructor of DummyLaserData -> set rotation duration time (time between two scans)
     */
    BinaryInterfaceDummyLaserData(const double rotationDurationInSec)
        : rotationDurationInSec(rotationDurationInSec),                         //One full rotation
          scanDurationInSec(rotationDurationInSec / 2.),                        //Part of the rotation scanning (180/360)
          beamDurationInSec(scanDurationInSec / nBeams),                        //One beam time
          rotationDurationChrono((int)(rotationDurationInSec * 1000. * 1000.)), //wait constant
          lastScan(std::chrono::system_clock::now())
    {
    }

    /*
     * Sleep until the next scan should be send and return the appropriate (constant) scan
     */
    ClientSensorLaserDatagram getWaitConstantScan(float ranges[], bool hasIntensity, float intensity[], unsigned int length)
    {
        std::this_thread::sleep_until(lastScan + rotationDurationChrono);
        lastScan = std::chrono::system_clock::now();

        ClientSensorLaserDatagram result;
        result.scanNum = ++currentScanNum;

        const std::chrono::duration<double> unixTimePoint = lastScan - std::chrono::system_clock::time_point();
        result.time_start = unixTimePoint.count();

        result.minRange = 0.001;
        result.maxRange = 30.0;
        result.angleStart = -M_PI * 135/180 ;
        //每次增加的角度
        result.angleInc = 270 / 180 * M_PI / float(1.0 * length);
        result.angleEnd = result.angleStart +  result.angleInc*length;
        result.duration_beam = beamDurationInSec;
        result.duration_scan = scanDurationInSec;
        result.duration_rotate = rotationDurationInSec;
        result.hasIntensities = hasIntensity;
        std::cout<<"increse = "<<result.angleInc<<"end = "<<result.angleEnd<<std::endl ;
        for (size_t i = 0; i <= length; ++i)
        {
            result.ranges[i] = ranges[i];
            if (hasIntensity)
            {
                result.intensities[i] = intensity[i];
            }
        }
        return result;
    }

private:
    double rotationDurationInSec;
    double scanDurationInSec;
    double beamDurationInSec;
    std::chrono::microseconds rotationDurationChrono;
    std::chrono::system_clock::time_point lastScan;
    uint16_t currentScanNum{0};
};

#endif /* BINARYINTERFACEDUMMYLASERSCAN */
