#include "bosch.h"
#include <iostream>

using namespace std;

int main()
{
    //login locator
    Bosch bosch ("172.17.0.1","admin","123");
    bosch.sessionLogin(8082);

//    //8.1.1 aboutModulesList: it's used to query a list of the used API modules and their versions.
//    bosch.aboutModulesList();
//    //8.1.2 aboutModulesComponent: queries the component type
//    bosch.aboutModulesComponent();
//
//    sleep(1);
//    //8.2.2 sessionLogout: Logout to the product
//    bosch.sessionLogout();
//    sleep(1);
    bosch.sessionLogin(8080);
//    sleep(2);

//    //8.2.3 sessionRefresh: Refreshes the given session, resetting its expiration timer. This works as a heartbeat mechanism,
//    bosch.sessionRefresh();
//    //8.2.4 sessionGroupInfo: Requests the user groups to which the session’s user belongs.
//    sleep(1);
//    bosch.sessionGroupInfo();
//    //8.2.5 sessionList: returns a list of current session info objects
//    bosch.sessionList();
//    //8.2.6 sessionRevoke:  revokes a session with a revokation id from an session info object
//    bosch.sessionRevoke("e9b44996-fdd4-4a43-a9c4-d473e57f4d6a");
//    //8.2.7 sessionRevokeAll: revokes all sessions including the one usedto call this method
//    bosch.sessionRevokeAll();
//    //8.2.8 sessionRevokeId: returns the revokation id of the calling session
//    bosch.sessionRevokeId();
//
//    //8.3 Diagnostic Information (Module: Diagnostic)
//    //8.3.1 Retrieves all available diagnostic entries from the buffer
//    bosch.diagnosticList();
//    //8.3.2 diagnosticClear Clears the diagnostic buffer, removing all stored diagnostic entries
//    bosch.diagnosticClear();

//    //8.4 Software Licensing (Module: LicensingFeature)
//    //8.4.1 licensingFeatureGetTrustedPlatformModuleInformation:Retrieves the Trusted Platform Module Information of the host.
//    bosch.licensingTPM();
//    //8.4.2 licensingFeatureGetHostId：  Retrieves the hardware-specific host ID.
//    bosch.licensingFeatureGetHostId();
//    //8.4.3 licensingFeatureSet: Sends a software license to the product element.
//    bosch.licensingFeatureSet("3-5186521-a96bf2387c70ba25384efc1a4e0389f1043c2d68c788de3e262400f5eeccd827.bin");
//    //8.4.4 licensingFeatureList: Retrieves information about the current license key.
//    bosch.licensingFeatureList();
//
//    //8.5 System Configuration
//    //8.5.1 configList
//    bosch.configList();
//    //8.5.2 configDefaultList: queries a list of all configurable parameters with their default values
//    bosch.configDefaultList();
//    //8.5.3 configSet : Sets all configurable parameters to the given values.
//    bosch.configSet_string("ClientSensor.laser.type", "simple", 8080);
//    bosch.configSet_num("ClientLaserMask.laser.minRange", 1.0, 8080);
//    bosch.configSet_bool("SupportRecovery.enableAutosave", true, 8080);
//    //TODO 将输入int 改为list，保证多条线的输入
//    //bool configSet_array(std::string key, std::vector<int> line_data);
//
//    //8.6.1 aboutBuildList： Returns human-readable version information.
//    bosch.aboutBuildList();
//
//    //8.7 Certificates
//    //8.7.1 certificateSet: Sets the certificates for the communication
//    //root/crt/key/rev: Container of media type application/CertificateFile
//    //bool certificateSet(std::string root, std::string crt, std::string key, std::string rev);
//
//    //8.8 Module System
//    //8.8.1 systemShutdown: The product element receiving this message is requested to shut down.
//    //bool systemShutdown();
//
//    //TODO Recording模块待测试
//    //9.1 Recording (Module: ClientRecording) .
//    //9.1.1 Requests to start recording laser scans of the environment, which can later beassem-bled into a map.
//    bosch.clientRecordingStart("testMap");
//    //9.1.2 clientRecordingStop:Requests to stop recording data.
//    sleep(3);
//    bosch.clientRecordingStop();
//    //9.1.3 clientRecordingStartVisualRecording: Requests to start recording laser scans of the environment(会构建一个临时虚拟地图)
//    bosch.clientRecordingStartVisualRecording("testVisualMap");
//    //9.1.4 clientRecordingStopVisualRecording
//    sleep(3);
//    bosch.clientRecordingStopVisualRecording();
//    //9.1.5 clientRecordingList Retrieves a list of all recordings stored by the Localization Client.
//    bosch.clientRecordingList();
//    //9.1.6 clientRecordingRename: Renames a recording stored by the Localization Client.
//    bosch.clientRecordingRename("testVisualMap", "mapTest");
//    //9.1.7 clientRecordingDelete Deletes a recording stored by the Localization Client.
//    //bool clientRecordingDelete("testVisualMap");
//    //9.1.8 clientRecordingSetCurrentPose: Provides the Localization Client with the current pose of the sensor while recording.
//    //bool clientRecordingSetCurrentPose(double x, double y, double angle);
//
//    //9.2 ClientMap
//    //9.2.1 clientMapStart: Requests to create a map from the data contained within a recording.(根据记录创建地图)
//    bosch.clientMapStart("mapTest", "map1");
//    //9.2.2 clientMapStop: Requests to stop map creation.(停止创建地图)
//    sleep(5);
//    bosch.clientMapStop();
//    //9.2.3 clientMapList: Retrieves all completed maps stored by the Localization Client.(检索定位客户端存储的所有完成的地图。)
//    bosch.clientMapList();
//    //9.2.4 clientMapRename: Renames a map stored by the Localization Client.(地图重命名)
//    bosch.clientMapRename("map1", "MAP");
//    //9.2.5 clientMapDelete: Delete a map stored by the Localization Client.(删除地图)
//    //bool clientMapDelete(std::string mapName);
//    //9.2.6 clientMapSend: Requests the Localization Client to send the map to the Map Server(发送地图到服务器)
//    bosch.clientMapSend("MAP");
//
//    //9.3 Client Localization (Module: ClientLocalization)
//    //9.3.1&9.3.2 flag clientLocalizationStart & Stop(开始/结束定位)
//    bosch.clientLocalizationSwitch(false);
//    sleep(1);
//    bosch.clientLocalizationSwitch(true);
//    sleep(1);
//    bosch.clientLocalizationSwitch(false);
//    //9.3.3 clientLocalizationSetSeed:Provides the Localization Client with
//    // an externally-derived pose estimate.(设置定位种子)
//    //bool clientSetSeed(double x, double y, double a);
//
//    //9.4  Manual Map Alignment (Module: ClientManualAlign)
//    //9.4.1 clientManualAlignStart : Requests to enter manual map alignment mode.
//    bosch.clientManualAlignStart();
//    //9.4.2 clientManualAlignStop : Requests to leave manual map alignment mode.
//    sleep(1);
//    bosch.clientManualAlignStop();
//    //9.4.3 clientManualAlignGetPointCloud: Retrieves a point cloud representation of a map to assist in manual
//    // map alignment.
//    bool clientManualAlignGetPointCloud(std::string clientMapName, int level);
//    //9.4.4 clientManualAlignSet :  Manually aligns a map, transforming it into the given reference frame.
//    bosch.clientManualAlignSet(1.02, 1.08, 3.14, "Quickstart_2021-12-27_13-59-48_built");
//    sleep(1);
//    bosch.clientManualAlignStop();

//    //9.5.1 clientGlobalAlignAddObservation: Adds a new landmark observation to the recording currently in progress.(记录中添加地标)
//    bool clientGlobalAlignAddObservation(ClientGlobalAlignObservation observation);
//    //9.5.2 clientGlobalAlignReplaceObservations:Replaces existing landmark observations within a saved recording.
//    bool clientGlobalAlignReplaceObservations(std::string recordingName,
//                                              std::vector<ClientGlobalAlignObservation> observation);
//    //9.5.3 clientGlobalAlignDeleteObservations: Deletes an existing landmark observation from a saved recording.
//    bool clientGlobalAlignDeleteObservations(std::string recordingName,
//                                             std::vector<int> observationIds);
//    //9.5.4 clientGlobalAlignListObservations: Retrieve the list of landmark observations which were added to an existing recording
//    bool clientGlobalAlignListObservations("testVisualMap");

//    //9.6.1 & 9.6.2 MaskingStart/Stop :Requests to enter/leave the control mode MASK(开启/结束mask模式)
//    bosch.clientLaserMaskingStart();
//    sleep(1);
//    bosch.clientLaserMaskingStop();
//    //9.6.3 clientLaserMaskingGetScan : Retrieves the next laser scan available to the Localization Client.
//    //检索本地化客户端可用的下一个激光扫描。
//    bosch.clientLaserMaskingGetScan();

//    //9.7 User Account Management (Module: ClientUser)
//    //9.7.2 clientUserAdd：Adds a new user account to the Localization Client.(添加用户)
//    bosch.clientUserAdd("Name", "123", "admin");
//    //9.7.3 clientUserDelete: Deletes a user account from the Localization Client.(删除用户)
//    bosch.clientUserDelete("Name");
//    //9.7.4 clientUserChangePassword： Changes a user’s password stored in the Localization Client.(修改定位客户端密码)
//    bosch.clientUserChangePassword("1234");
//    //9.7.5 clientUserChangeOwnPassword: this method is not restricted to the admin group, and can be used by all users
//    // to change their own passwords.(修改自己的密码)
//    bosch.clientUserChangeOwnPassword("123", "1234");
//    //9.7.6 clientUserList:Retrieves the list of all user accounts known to the Localization Client.(检索用户列表)
//    bosch.clientUserList();
//    //9.7.7 clientUserListGroup:Retrieves the list of all user groups known to the Localization Client.(检索用户组)
//    bosch.clientUserListGroup();

//    //9.8 Map Expansion (Module: ClientExpandMap)
//    //9.8.1 clientExpandMapEnable: Enables map expansion and requests to enter the control mode EXPANDMAP
//    bosch.clientExpandMapEnable("Quickstart_2022-01-19_17-03-58_built");
//    //clientExpandMapDisable: Disables map expansion and requests to leave the control mode EXPANDMAP
//    bosch.clientExpandMapDisable();
//    //clientExpandMapAddOverwriteZone
//    std::vector<Position2D> zone ;
//    zone.push_back(Position2D(1.00 , 2.00)) ;
//    zone.push_back(Position2D(1.00 , 2.00)) ;
//    zone.push_back(Position2D(1.00 , 2.00)) ;
//    bosch.clientExpandMapAddOverwriteZone("1" ,1 , zone) ;
//    //9.8.4 clientExpandMapDeleteOverwriteZones: Deletes overwrite zones in a saved recording
//    bosch.clientExpandMapDeleteOverwriteZones("1_2022-01-19_17-25-39", 1);
//    //clientExpandMapReplaceOverwriteZones
//    bosch.clientExpandMapReplaceOverwriteZones("1_2022-01-19_17-25-39", 1,2);
//    //9.8.6 clientExpandMapListOverwriteZones: Lists overwrite zones in an existing and finalized recording.
//    bosch.clientExpandMapListOverwriteZones("1_2022-01-19_17-25-39");
//    //9.8.7 clientExpandMapGetPriorMapPointCloud: Gets the prior map in an existing and finalized recording.
//    bosch.clientExpandMapGetPriorMapPointCloud("1_2022-01-19_17-25-39");
//
//    //10.1 Server-side Map Management (Module: ServerMap)
//    //10.1.1 serverMapGetInfo : Returns all information about a given map
//    bosch.serverMapGetInfo("Quickstart_2021-12-27_13-59-48_built");
//    //10.1.2 serverMapList : Requests a list of all maps mananged by the map server.
//    bosch.serverMapList();
//    //10.1.3 serverMapRename:Renames a map managed by the Map Server.
//    bosch.serverMapRename("Quickstart_2021-12-27_13-59-48_built", "mapServer");
//    //10.1.4 serverMapDelete: Deletes a map managed by the Map Server.
//    //bool serverMapDelete(std::string serverMapName);
//    //10.1.5 serverMapGetPointCloud: Retrieves a point cloud representation of a map managed by the server.
//    bosch.serverMapGetPointCloud("mapServer");
//    //10.1.6 serverMapGetImage: Retrieves a raster image of the map with a given image size.
//    bosch.serverMapGetImage(100, 100, 10, "Quickstart_2021-12-27_13-59-48_built");
//    //10.1.7 serverMapGetImageWithResolution :Retrieves a raster image of the map with a given image resolution.
//    //bool serverMapGetImageWithResolution(std::string serverMapName, double resolution);
//    //10.1.8 serverMapGetMapThumbnail: Retrieves a thumbnail-sized raster image of the map.(检索缩略图)
//    //bool serverMapGetMapThumbnail(std::string serverMapName, int level);
//
//    //10.2 User Account Management (Module:ServerUser)
//    //10.2.2 serverUserAdd：Adds a new user account to the Map Server.(添加用户)
//    bosch.serverUserAdd("admin","123", "admin");
//    //10.2.3 serverUserDelete: Deletes a user account from tMap Server.(删除用户)
//    //bool serverUserDelete(std::string username);
//    //10.2.4 serverUserChangePassword： Changes a user’s password stored in Map Server.(修改地图服务器端密码)
//    bosch.serverUserChangePassword("1234");
//    //10.2.5 serverUserChangeOwnPassword: this method is not restricted to the admin group, and can be used by all users
//    // to change their own passwords.(修改自己的密码)
//    bosch.serverUserChangeOwnPassword("123", "1234");
//    //10.2.6 serverUserList:Retrieves the list of all user accounts known to Map Server.(检索用户列表)
//    bosch.serverUserList();
//    //10.2.7 serverUserListGroup:Retrieves the list of all user groups known to Map Server.(检索用户组)
//    bosch.serverUserListGroup();
//
//    //session_id don't match,change 8084
//    //11.1 Support Report(Module: SupportReport)
//    bosch.supportReportSetDescription("test");
//    //11.1.1 supportReportCreate Creates a new support report that documents the current system state.（创建新的报告）
//    //responseCode = 1;
//    bosch.supportReportCreate();
//    //11.2 supportReportCreateMinimal: Creates a new minimal support report that provides a limited documentation of the current system state.（最低支持报告）
//    bosch.supportReportCreateMinimal();
//    //11.3 supportReportSetDescription: Stores a support report description, which can include additional support informationsupplied by a user.(设置报告描述)
//    bosch.supportReportSetDescription("test");
//    //11.4 supportReportList: Retrieves a list of all known support reports.(检索报告列表)
//    bosch.supportReportList();
//    //11.1.5 bool supportReportCreateandgetpath();: Creates a new support report that documents the current system state.()
//    bosch.supportReportCreateandgetpath();
//    //11.1.6 supportReportDelete: Deletes the specified support report.
//    bool supportReportDelete(std::string reportName);

}