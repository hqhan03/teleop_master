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
    float wristQuaternion[4]; // w, x, y, z
    float fingerFlexion[20]; // Thumb, Index, Middle, Ring, Pinky (MCP_Sp, MCP_St, PIP, DIP)
    float fingertipPos[15]; // 5 fingertips × (X, Y, Z) — Thumb, Index, Middle, Ring, Pinky
};
#pragma pack(pop)

enum class ConnectionType : int {
    ConnectionType_Invalid = 0, ConnectionType_Integrated, ConnectionType_Local, ConnectionType_Remote
};

enum class ClientReturnCode : int {
    ClientReturnCode_Success = 0, ClientReturnCode_FailedToInitialize, ClientReturnCode_FailedToConnect
};

class TeleopMasterClient : public SDKClientPlatformSpecific
{
public:
    TeleopMasterClient();
    ~TeleopMasterClient();
    ClientReturnCode Initialize();
    ClientReturnCode InitializeSDK();
    void Run();
    ClientReturnCode ShutDown();
    ClientReturnCode RegisterAllCallbacks();

    // UDP 관련 함수
    bool InitializeUDP(const char* ip, int port);
    void SendUDPData(float posX, float posY, float posZ, float rotW, float rotX, float rotY, float rotZ);

    // 콜백 함수
    static void OnTrackerStreamCallback(const TrackerStreamInfo* const p_TrackerStreamInfo);
    static void OnErgonomicsCallback(const ErgonomicsStream* const p_Ergo);
    static void OnLandscapeCallback(const Landscape* const p_Landscape);
    static void OnRawSkeletonStreamCallback(const SkeletonStreamInfo* const p_Info);

protected:
    ClientReturnCode Connect();

    static TeleopMasterClient* s_Instance;
    bool m_Running;
    ConnectionType m_ConnectionType;

    // 데이터 보호 및 저장
    std::mutex m_DataMutex;
    TrackerData m_WristTracker;
    ErgonomicsData m_RightGloveData;

    uint32_t m_RightGloveID = 0;
    uint32_t m_FrameCounter = 0;

    // Raw skeleton fingertip data
    bool m_TipNodeIdsResolved = false;
    uint32_t m_TipNodeIndices[5] = {0}; // Indices into SkeletonNode array for 5 fingertip nodes
    float m_FingertipPositions[15] = {0}; // XYZ for 5 fingertips (Thumb, Index, Middle, Ring, Pinky)

    // Tracker Calibration (Zeroing)
    bool m_Calibrated = false;
    float m_PosOffsetX = 0.0f;
    float m_PosOffsetY = 0.0f;
    float m_PosOffsetZ = 0.0f;
    
    // Quaternion offset (inverse of the zero rotation)
    float m_RotOffsetW = 1.0f;
    float m_RotOffsetX = 0.0f;
    float m_RotOffsetY = 0.0f;
    float m_RotOffsetZ = 0.0f;

    // UDP 통신 멤버
    SOCKET m_Socket;
    sockaddr_in m_DestAddr;
    bool m_UdpInitialized = false;
};

#endif
