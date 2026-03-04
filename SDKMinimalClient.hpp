#ifndef _SDK_MINIMAL_CLIENT_HPP_
#define _SDK_MINIMAL_CLIENT_HPP_

#include "ClientPlatformSpecific.hpp"
#include "ManusSDK.h"
#include <mutex>
#include <vector>

// UDP 통신을 위한 Winsock 라이브러리
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")

// 1. ROS2 수신단과 맞출 데이터 패킷 구조체 (바이트 정렬 고정)
#pragma pack(push, 1)
struct HandDataPacket {
    uint32_t frame;
    float wristPos[3];   // X, Y, Z (meters)
    float wristEuler[3]; // Roll, Pitch, Yaw (degrees)
    float fingerFlexion[15]; // Thumb(MCP,PIP,DIP), Index(MCP,PIP,DIP)... 순서
};
#pragma pack(pop)

enum class ConnectionType : int {
    ConnectionType_Invalid = 0, ConnectionType_Integrated, ConnectionType_Local, ConnectionType_Remote
};

enum class ClientReturnCode : int {
    ClientReturnCode_Success = 0, ClientReturnCode_FailedToInitialize, ClientReturnCode_FailedToConnect
};

class SDKMinimalClient : public SDKClientPlatformSpecific
{
public:
    SDKMinimalClient();
    ~SDKMinimalClient();
    ClientReturnCode Initialize();
    ClientReturnCode InitializeSDK();
    void Run();
    ClientReturnCode ShutDown();
    ClientReturnCode RegisterAllCallbacks();

    // UDP 관련 함수
    bool InitializeUDP(const char* ip, int port);
    void SendUDPData();

    // 콜백 함수
    static void OnTrackerStreamCallback(const TrackerStreamInfo* const p_TrackerStreamInfo);
    static void OnErgonomicsCallback(const ErgonomicsStream* const p_Ergo);
    static void OnLandscapeCallback(const Landscape* const p_Landscape);

protected:
    ClientReturnCode Connect();

    static SDKMinimalClient* s_Instance;
    bool m_Running;
    ConnectionType m_ConnectionType;

    // 데이터 보호 및 저장
    std::mutex m_DataMutex;
    TrackerData m_WristTracker;
    ErgonomicsData m_RightGloveData;

    uint32_t m_RightGloveID = 0;
    uint32_t m_FrameCounter = 0;

    // UDP 통신 멤버
    SOCKET m_Socket;
    sockaddr_in m_DestAddr;
    bool m_UdpInitialized = false;
};

#endif