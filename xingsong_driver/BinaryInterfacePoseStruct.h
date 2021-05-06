/*
 * File: BinaryInterfacePoseStruct.h
 * Created On: 2019-03-13
 * Copyright © 2019 Bosch Rexroth AG
*/

#ifndef BINARYINTERFACESTRUCTS_H
#define BINARYINTERFACESTRUCTS_H

/*
 * structure containing the exact message from the ClientLocalizationPose Interface
 */
struct __attribute__ ((packed)) PoseMessage
{
    double   age       {0.};
    double   timestamp {0.};
    uint64_t uniqueId  {0};
    int32_t  locState  {0};
    double   x         {0.};
    double   y         {0.};
    double   yaw       {0.};
    double   z         {0.};
    double   qw        {0.};
    double   qx        {0.};
    double   qy        {0.};
    double   qz        {0.};
    uint64_t epoch     {0};
    double   x_odo     {0.};
    double   y_odo     {0.};
    double   yaw_odo   {0.};
};

#endif /* BINARYINTERFACESTRUCTS_H */
