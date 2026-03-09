#include "TeleopMasterClient.hpp"
#include "ManusSDKTypes.h"
#include <iostream>
#include <thread>
#include <cmath>
#include "ClientLogging.hpp"

using ManusSDK::ClientLog;
TeleopMasterClient* TeleopMasterClient::s_Instance = nullptr;

// Using raw Quaternion directly

int main(int argc, char* argv[]) {
    TeleopMasterClient t_Client;
    if (t_Client.Initialize() != ClientReturnCode::ClientReturnCode_Success) return -1;
    t_Client.Run();
    t_Client.ShutDown();
    return 0;
}

TeleopMasterClient::TeleopMasterClient() : m_Running(true), m_ConnectionType(ConnectionType::ConnectionType_Local) {
    s_Instance = this;
    TrackerData_Init(&m_WristTracker);
    ErgonomicsData_Init(&m_RightGloveData);
}

TeleopMasterClient::~TeleopMasterClient() { s_Instance = nullptr; }

// UDP 珥덇린??(Initialize UDP)
bool TeleopMasterClient::InitializeUDP(const char* ip, int port) {
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) return false;
    m_Socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (m_Socket == INVALID_SOCKET) return false;

    m_DestAddr.sin_family = AF_INET;
    m_DestAddr.sin_port = htons(port);
    inet_pton(AF_INET, ip, &m_DestAddr.sin_addr);
    m_UdpInitialized = true;
    return true;
}

// UDP ?占쎌씠???占쎌넚
void TeleopMasterClient::SendUDPData() {
    if (!m_UdpInitialized) return;
    HandDataPacket packet;
    packet.frame = m_FrameCounter;

    packet.wristPos[0] = m_WristTracker.position.x;
    packet.wristPos[1] = m_WristTracker.position.y;
    packet.wristPos[2] = m_WristTracker.position.z;

    packet.wristQuaternion[0] = m_WristTracker.rotation.w;
    packet.wristQuaternion[1] = m_WristTracker.rotation.x;
    packet.wristQuaternion[2] = m_WristTracker.rotation.y;
    packet.wristQuaternion[3] = m_WristTracker.rotation.z;

    int offset = 20; // 오른손 관절각 데이터 오프셋 (Right Hand Joint Offset)
    for (int i = 0; i < 5; i++) {
        packet.fingerFlexion[i * 4 + 0] = m_RightGloveData.data[offset + (i * 4) + 0]; // MCP Spread
        packet.fingerFlexion[i * 4 + 1] = m_RightGloveData.data[offset + (i * 4) + 1]; // MCP Stretch
        packet.fingerFlexion[i * 4 + 2] = m_RightGloveData.data[offset + (i * 4) + 2]; // PIP
        packet.fingerFlexion[i * 4 + 3] = m_RightGloveData.data[offset + (i * 4) + 3]; // DIP
    }

    sendto(m_Socket, (char*)&packet, sizeof(packet), 0, (sockaddr*)&m_DestAddr, sizeof(m_DestAddr));
}

ClientReturnCode TeleopMasterClient::Initialize() {
    if (!PlatformSpecificInitialization()) return ClientReturnCode::ClientReturnCode_FailedToInitialize;
    return InitializeSDK();
}

ClientReturnCode TeleopMasterClient::InitializeSDK() {
    if (CoreSdk_InitializeCore() != SDKReturnCode::SDKReturnCode_Success) return ClientReturnCode::ClientReturnCode_FailedToInitialize;
    RegisterAllCallbacks();
    CoordinateSystemVUH t_VUH = { AxisView::AxisView_XFromViewer, AxisPolarity::AxisPolarity_PositiveZ, Side::Side_Right, 1.0f };
    CoreSdk_InitializeCoordinateSystemWithVUH(t_VUH, true);
    return ClientReturnCode::ClientReturnCode_Success;
}

ClientReturnCode TeleopMasterClient::RegisterAllCallbacks() {
    CoreSdk_RegisterCallbackForTrackerStream(*OnTrackerStreamCallback);
    CoreSdk_RegisterCallbackForErgonomicsStream(*OnErgonomicsCallback);
    CoreSdk_RegisterCallbackForLandscapeStream(*OnLandscapeCallback);
    return ClientReturnCode::ClientReturnCode_Success;
}

void TeleopMasterClient::OnLandscapeCallback(const Landscape* const p_Landscape) {
    if (!s_Instance) return;
    std::lock_guard<std::mutex> lock(s_Instance->m_DataMutex);
    for (uint32_t i = 0; i < p_Landscape->gloveDevices.gloveCount; i++) {
        if (p_Landscape->gloveDevices.gloves[i].side == Side_Right)
            s_Instance->m_RightGloveID = p_Landscape->gloveDevices.gloves[i].id;
    }
}

void TeleopMasterClient::OnErgonomicsCallback(const ErgonomicsStream* const p_Ergo) {
    if (!s_Instance) return;
    std::lock_guard<std::mutex> lock(s_Instance->m_DataMutex);
    for (uint32_t i = 0; i < p_Ergo->dataCount; i++) {
        if (p_Ergo->data[i].id == s_Instance->m_RightGloveID) s_Instance->m_RightGloveData = p_Ergo->data[i];
    }
}

void TeleopMasterClient::OnTrackerStreamCallback(const TrackerStreamInfo* const p_Info) {
    if (s_Instance && p_Info->trackerCount > 0) {
        std::lock_guard<std::mutex> lock(s_Instance->m_DataMutex);
        bool trackerFound = false;
        TrackerData t_TrackerData;
        for (uint32_t i = 0; i < p_Info->trackerCount; i++) {
            if (CoreSdk_GetTrackerData(i, &t_TrackerData) == SDKReturnCode::SDKReturnCode_Success) {
                if (t_TrackerData.trackerType == TrackerType::TrackerType_RightHand) {
                    s_Instance->m_WristTracker = t_TrackerData;
                    trackerFound = true;
                    break;
                }
            }
        }
        // Fallback to the first tracker if RightHand is not explicitly found
        if (!trackerFound && CoreSdk_GetTrackerData(0, &t_TrackerData) == SDKReturnCode::SDKReturnCode_Success) {
            s_Instance->m_WristTracker = t_TrackerData;
        }
    }
}

void TeleopMasterClient::Run() {
    // [중요] 타겟 수신 PC(예: Ubuntu)의 실제 IP로 변경하세요.
    if (!InitializeUDP("192.168.0.112", 12345)) {
        ClientLog::error("Failed to initialize UDP.");
        return;
    }

    while (Connect() != ClientReturnCode::ClientReturnCode_Success) std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    while (m_Running) {
        {
            std::lock_guard<std::mutex> lock(m_DataMutex);
            SendUDPData(); // 실시간 패킷 전송

            system("cls");
            printf("=== [KAIST NREL] MANUS -> ROS2 Humble (UDP 50Hz) ===\n");
            printf("[Wrist] Pos: X:%.3f Y:%.3f Z:%.3f | Quat: W:%.3f X:%.3f Y:%.3f Z:%.3f\n",
                m_WristTracker.position.x, m_WristTracker.position.y, m_WristTracker.position.z,
                m_WristTracker.rotation.w, m_WristTracker.rotation.x, m_WristTracker.rotation.y, m_WristTracker.rotation.z);

            if (m_RightGloveID != 0) {
                printf("[Glove ID: 0x%X] Sending 15 Finger Joints...\n", m_RightGloveID);
                printf("  Thumb:  CMC_Fl/Ex=%.2f CMC_Ab/Ad=%.2f MCP_Fl/Ex=%.2f IP_Fl/Ex=%.2f\n",
                    m_RightGloveData.data[20], m_RightGloveData.data[21], m_RightGloveData.data[22], m_RightGloveData.data[23]);
                printf("  Index:  MCP_Ab/Ad=%.2f MCP_Fl/Ex=%.2f PIP_Fl/Ex=%.2f DIP_Fl/Ex=%.2f\n",
                    m_RightGloveData.data[24], m_RightGloveData.data[25], m_RightGloveData.data[26], m_RightGloveData.data[27]);
                printf("  Middle: MCP_Ab/Ad=%.2f MCP_Fl/Ex=%.2f PIP_Fl/Ex=%.2f DIP_Fl/Ex=%.2f\n",
                    m_RightGloveData.data[28], m_RightGloveData.data[29], m_RightGloveData.data[30], m_RightGloveData.data[31]);
                printf("  Ring:   MCP_Ab/Ad=%.2f MCP_Fl/Ex=%.2f PIP_Fl/Ex=%.2f DIP_Fl/Ex=%.2f\n",
                    m_RightGloveData.data[32], m_RightGloveData.data[33], m_RightGloveData.data[34], m_RightGloveData.data[35]);
                printf("  Pinky:  MCP_Ab/Ad=%.2f MCP_Fl/Ex=%.2f PIP_Fl/Ex=%.2f DIP_Fl/Ex=%.2f\n",
                    m_RightGloveData.data[36], m_RightGloveData.data[37], m_RightGloveData.data[38], m_RightGloveData.data[39]);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        if (GetKeyDown(' ')) m_Running = false;
        m_FrameCounter++;
    }
}

ClientReturnCode TeleopMasterClient::Connect() {
    if (CoreSdk_LookForHosts(1, true) != SDKReturnCode::SDKReturnCode_Success) return ClientReturnCode::ClientReturnCode_FailedToConnect;
    uint32_t count = 0;
    CoreSdk_GetNumberOfAvailableHostsFound(&count);
    if (count == 0) return ClientReturnCode::ClientReturnCode_FailedToConnect;
    ManusHost host;
    CoreSdk_GetAvailableHostsFound(&host, 1);
    return (CoreSdk_ConnectToHost(host) == SDKReturnCode::SDKReturnCode_Success) ? ClientReturnCode::ClientReturnCode_Success : ClientReturnCode::ClientReturnCode_FailedToConnect;
}

ClientReturnCode TeleopMasterClient::ShutDown() {
    if (m_UdpInitialized) { closesocket(m_Socket); WSACleanup(); }
    CoreSdk_ShutDown();
    PlatformSpecificShutdown();
    return ClientReturnCode::ClientReturnCode_Success;
}


