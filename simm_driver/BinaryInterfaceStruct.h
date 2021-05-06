#ifndef BINARY_INTERFACE_STRUCT_H
#define BINARY_INTERFACE_STRUCT_H

#include "json.hpp"
#include <fstream>  //for licensingFeatureGetHostId
#include <iostream> //for licensingFeatureGetHostId
#include <unistd.h> //for sleep() at   void Bosch::startLocal()
#include <vector>

/* *****************************Struct********************************************** */
typedef struct
{
  bool calid;
  int64_t time;
  double resolution; //number, must be one of 1, 100, 1000, 1000000, or 1000000000
} Timestamp;

typedef struct
{
  double x;  //absolute x coordinate in map frame
  double y;  //absolute y coordinate in map frame
  double z;  //z coordinate
  double qw; //quaternion w (real) coordinate
  double qx; //quaternion x coordinate
  double qy; //quaternion y coordinate
  double qz; //quaternion z coordinate
} Pose3D;

typedef struct
{
  double x;  //absolute x coordinate in map frame
  double y;  //absolute y coordinate in map frame
  double z;  //z coordinate
  double qw; //quaternion w (real) coordinate
  double qx; //quaternion x coordinate
  double qy; //quaternion y coordinate
  double qz; //quaternion z coordinate
} Transform3D;

typedef struct
{
  double x; //x坐标
  double y; //y坐标
  double a; //a角度
} Pose2D;

typedef struct
{
  float x;
  float y;
  float a;
} Pose2DSingle;

typedef struct
{
  double x;
  double y;
} Position2D;

typedef struct
{
  float x;
  float y;
} Position2DSingle;

typedef struct
{
  std::string contentEncoding;  //文档编码
  std::string contentMediaType; //文档格式
  std::string content;          //文档内容
} Container;

typedef struct
{
  double age{0.};       //The time passed between receiving the most recent laser scan and the generation of this Datagram.
  double timestamp{0.}; //The time at which the Localization Client received the laser scan used to generate this Datagram.
  uint64_t uniqueId{0}; //Currently unused; this ﬁeld is reserved to match a uniqueId provided via the ClientSensorLaser interface.
  int32_t locState{0};  //The localization status.
  double x{0.};         //The estimated Cartesian (x,y)-coordinates and yaw angle of the sensor, given in the map reference frame.
  double y{0.};         //The estimated Cartesian (x,y)-coordinates and yaw angle of the sensor, given in the map reference frame.
  double yaw{0.};       //The estimated Cartesian (x,y)-coordinates and yaw angle of the sensor, given in the map reference frame.
  double z{0.};         //This value is unused in this version of the API and will always be 0.
  double qw{0.};        // The sensor orientation given as quaternions.
  double qx{0.};        // The sensor orientation given as quaternions.
  double qy{0.};        // The sensor orientation given as quaternions.
  double qz{0.};        // The sensor orientation given as quaternions.
  uint64_t epoch{0};    // Only Datagrams with the same epoch can be treated as locally precise and coherent.
  double x_odo{0.};     // The estimated Cartesian (x,y)-coordinates and yaw angle of the sensor, given in an arbitrary, relative reference frame.
  double y_odo{0.};     // The estimated Cartesian (x,y)-coordinates and yaw angle of the sensor, given in an arbitrary, relative reference frame.
  double yaw_odo{0.};   // The estimated Cartesian (x,y)-coordinates and yaw angle of the sensor, given in an arbitrary, relative reference frame.
} ClientLocalizationPoseDatagram;

typedef struct
{
  double timestamp{0.};
  uint64_t uniqueId{0};
  int32_t locState{0};
  double x{0.};
  double y{0.};
  double yaw{0.};
  double delay{0.};
  std::vector<Position2DSingle> scan;
} ClientLocalizationVisualizationDatagram;

typedef struct
{
  double timestamp;
  uint64_t visualizationId;
  int32_t status;
  double x;
  double y;
  double yaw;
  double distanceToLastLC;
  double delay;
  double progress;
  std::vector<Position2DSingle> scan;
  std::vector<Pose2DSingle> pathPoses;
  std::vector<int32_t> pathTypes;

} ClientVisualizationDatagram;

typedef struct
{
  Pose2DSingle pose;
  u_int64_t type;
  bool hasOrientation;
  std::vector<u_int8_t> name;
} ClientGlobalAlignLandmarkVisualizationInformation;

typedef struct
{
  u_int32_t poseIndex;
  u_int32_t landmarkIndex;
} ClientGlobalAlignLandmarkObservationNotice; // Encodes the fact that a landmark with the given index was observed from a pose with the given index.

typedef struct
{
  double timestamp;
  uint64_t visualizationId;
  std::vector<Pose2DSingle> poses;
  ClientGlobalAlignLandmarkVisualizationInformation landmarks;
  ClientGlobalAlignLandmarkObservationNotice observations;
} ClientGlobalAlignVisualizationDatagram;

typedef struct
{
  double timestamp; //timestamp (UTC [sec]) of corr. scan
  uint32_t odom_number;
  uint64_t epoch;
  double x;
  double y;
  double yaw;
  double v_x;
  double v_y;
  double omega;
  bool velocitySet;
} ClientSensorOdometryDatagram;

/**
 * 该数据报包含由车辆的车载惯性测量单元测量的数据。 要指定从测量单位框架到激光框架的转换，用户必须设置相应的配置项。 请注意，只要连续的消息使用相同的
 * 帧，所有度量都可以作为任意参考帧。
 */
typedef struct
{
  double timestamp; //timestamp (UTC [sec]) of corr. scan
  u_int32_t imuNumber;
  uint64_t epoch;
  double orientation_qw; // The orientation of the IMU at the time timestamp, in quaternion representation.
  double orientation_qx; // The orientation of the IMU at the time timestamp, in quaternion representation.
  double orientation_qy; // The orientation of the IMU at the time timestamp, in quaternion representation.
  double orientation_qz; // The orientation of the IMU at the time timestamp, in quaternion representation.
  double omega_x;        // The angular velocities measured bythe IMU around the three axes at the time timestamp.
  double omega_y;        // The angular velocities measured bythe IMU around the three axes at the time timestamp.
  double omega_z;        // The angular velocities measured bythe IMU around the three axes at the time timestamp.
  double a_x;            // The linear acceleraration measured by the IMU around the three axes at the time timestamp.
  double a_y;            // The linear acceleraration measured by the IMU around the three axes at the time timestamp.
  double a_z;            // The linear acceleraration measured by the IMU around the three axes at the time timestamp.
  bool orientationSet;   // Indicates whether or not the IMU’s orientation is known.
  bool linearAccSet;     // IndicateswhetherornottheIMU’slinearaccelerationsareknown.
} ClientSensorInertialMeasurementUnitDatagram;

typedef enum
{
  FIX_ALL,                              //Calibration is fixed for orientation and translation
  OPTIMIZE_ALL,                         //Calibration is optimized for orientation and translation
  OPTIMIZE_TRANSLATION_FIX_ORIENTATION, //Calibration is fixed in orientation but optimized in translation
  FIX_TRANSLATION_OPTIMIZE_ORIENTATION  //Calibration is optimized in orientation but fixed in translation
} GlobalAlignCalibrationType;

// map TaskState values to JSON as int
NLOHMANN_JSON_SERIALIZE_ENUM(GlobalAlignCalibrationType, {{FIX_ALL, 0},      //Observation will not be used for global alignment.
                                                          {OPTIMIZE_ALL, 1}, //Observation is used for global alignment. Observations of this type should be precise
                                                          {OPTIMIZE_TRANSLATION_FIX_ORIENTATION, 2},
                                                          {FIX_TRANSLATION_OPTIMIZE_ORIENTATION, 3}})

typedef enum
{
  ANNOTATION, //Observation will not be used for global alignment.
  REFERENCE,  //Observation is used for global alignment. Observations of this type should be precise
  IGNORE      //All observations of this type are ignored
} GlobalAlignObservationType;

// map TaskState values to JSON as int
NLOHMANN_JSON_SERIALIZE_ENUM(GlobalAlignObservationType, {{ANNOTATION, 0}, //Observation will not be used for global alignment.
                                                          {REFERENCE, 1},  //Observation is used for global alignment. Observations of this type should be precise
                                                          {IGNORE, 2}})

typedef struct
{
  std::string landmarkName;      //unique ClientGlobalAlignLandmarkName identifier of the observed landmark
  std::string sensorName;        //unique ClientGlobalAlignSensorName identifier for the sensor that made the observation
  Pose2D laserStaticCalibSensor; //perceiving the landmark sensor
  Pose2D sensorLandmark;         //Observed pose of the landmark in the sensor coordinate system
  Pose2D mapLandmark;            //Precise pose of the landmark in the map coordinate system
  int obType;                    //observation type
  int caliType;                  //calibration type
  bool hasOrientation;           //indicates whether the orientation of the landmark is known
} ClientGlobalAlignObservation;

#endif //BINARY_INTERFACE_STRUCT_H