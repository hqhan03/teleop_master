#include "TeleopMasterClient.hpp"
#include "ManusSDKTypes.h"
#include <iostream>
#include <thread>
#include <cmath>
#include "ClientLogging.hpp"

using ManusSDK::ClientLog;

// 싱글톤(Singleton) 인스턴스 전역 포인터 (콜백 함수에서 접근하기 위함)
TeleopMasterClient* TeleopMasterClient::s_Instance = nullptr;

int main(int argc, char* argv[]) {
    TeleopMasterClient t_Client;
    
    // 1. 클라이언트(SDK 및 소켓) 초기화
    if (t_Client.Initialize() != ClientReturnCode::ClientReturnCode_Success) return -1;
    
    // 2. 메인 루프 실행 (데이터 수신 및 UDP 전송)
    t_Client.Run();
    
    // 3. 종료 시 리소스 정리
    t_Client.ShutDown();
    return 0;
}

TeleopMasterClient::TeleopMasterClient() : m_Running(true), m_ConnectionType(ConnectionType::ConnectionType_Local) {
    s_Instance = this;
    TrackerData_Init(&m_WristTracker);
    ErgonomicsData_Init(&m_RightGloveData);
}

TeleopMasterClient::~TeleopMasterClient() { s_Instance = nullptr; }


// UDP 소켓을 초기화하고 목적지(수신 서버)의 IP 및 포트를 설정
bool TeleopMasterClient::InitializeUDP(const char* ip, int port) {
    WSADATA wsaData;
    // Winsock 초기화
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) return false;
    
    // UDP 데이터그램 소켓 생성
    m_Socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (m_Socket == INVALID_SOCKET) return false;

    // 목적지 주소 및 포트 바인딩 설정
    m_DestAddr.sin_family = AF_INET;
    m_DestAddr.sin_port = htons(port);
    inet_pton(AF_INET, ip, &m_DestAddr.sin_addr);
    
    m_UdpInitialized = true;
    return true;
}


// 수집된 Hand/Wrist 데이터를 UDP 패킷 형태로 직렬화하여 송신
void TeleopMasterClient::SendUDPData() {
    if (!m_UdpInitialized) return;
    
    HandDataPacket packet;
    packet.frame = m_FrameCounter;

    // 1. 손목 트래커 위치 데이터 (X, Y, Z)
    packet.wristPos[0] = m_WristTracker.position.x;
    packet.wristPos[1] = m_WristTracker.position.y;
    packet.wristPos[2] = m_WristTracker.position.z;

    // 2. 손목 트래커 회전 데이터 (W, X, Y, Z 쿼터니언)
    packet.wristQuaternion[0] = m_WristTracker.rotation.w;
    packet.wristQuaternion[1] = m_WristTracker.rotation.x;
    packet.wristQuaternion[2] = m_WristTracker.rotation.y;
    packet.wristQuaternion[3] = m_WristTracker.rotation.z;

    // 3. 우측 글로브 손가락 관절 데이터 추출 (오프셋 20부터 각 손가락당 4개의 관절 데이터 존재)
    int offset = 20;
    for (int i = 0; i < 5; i++) {
        // 엄지부터 새끼손가락까지 (Spread/CMC, Stretch/MCP, PIP, DIP) 순서로 구조체에 복사
        packet.fingerFlexion[i * 4 + 0] = m_RightGloveData.data[offset + (i * 4) + 0];
        packet.fingerFlexion[i * 4 + 1] = m_RightGloveData.data[offset + (i * 4) + 1];
        packet.fingerFlexion[i * 4 + 2] = m_RightGloveData.data[offset + (i * 4) + 2];
        packet.fingerFlexion[i * 4 + 3] = m_RightGloveData.data[offset + (i * 4) + 3];
    }

    // UDP 데이터 전송
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

// [Landscape 콜백] 연결된 기기들의 환경(Landscape) 정보를 받아와 우측 글로브의 ID를 획득
void TeleopMasterClient::OnLandscapeCallback(const Landscape* const p_Landscape) {
    if (!s_Instance) return;
    std::lock_guard<std::mutex> lock(s_Instance->m_DataMutex);
    for (uint32_t i = 0; i < p_Landscape->gloveDevices.gloveCount; i++) {
        if (p_Landscape->gloveDevices.gloves[i].side == Side_Right)
            s_Instance->m_RightGloveID = p_Landscape->gloveDevices.gloves[i].id;
    }
}

// [Ergonomics 콜백] 인체공학적(Ergonomics) 관절 각도 데이터를 실시간으로 수신
void TeleopMasterClient::OnErgonomicsCallback(const ErgonomicsStream* const p_Ergo) {
    if (!s_Instance) return;
    std::lock_guard<std::mutex> lock(s_Instance->m_DataMutex);
    for (uint32_t i = 0; i < p_Ergo->dataCount; i++) {
        if (p_Ergo->data[i].id == s_Instance->m_RightGloveID) s_Instance->m_RightGloveData = p_Ergo->data[i];
    }
}

// [Tracker 콜백] VIVE 트래커 등에서 발생하는 위치 및 회전 데이터를 실시간으로 수신
void TeleopMasterClient::OnTrackerStreamCallback(const TrackerStreamInfo* const p_Info) {
    if (s_Instance && p_Info->trackerCount > 0) {
        std::lock_guard<std::mutex> lock(s_Instance->m_DataMutex);
        bool trackerFound = false;
        TrackerData t_TrackerData;
        
        // 연결된 트래커 중 우측 손(RightHand) 트래커를 찾아 데이터를 저장
        for (uint32_t i = 0; i < p_Info->trackerCount; i++) {
            if (CoreSdk_GetTrackerData(i, &t_TrackerData) == SDKReturnCode::SDKReturnCode_Success) {
                if (t_TrackerData.trackerType == TrackerType::TrackerType_RightHand) {
                    s_Instance->m_WristTracker = t_TrackerData;
                    trackerFound = true;
                    break;
                }
            }
        }
        
        // 명시적인 RightHand 트래커가 없을 경우 첫 번째 트래커를 사용 (Fallback)
        if (!trackerFound && CoreSdk_GetTrackerData(0, &t_TrackerData) == SDKReturnCode::SDKReturnCode_Success) {
            s_Instance->m_WristTracker = t_TrackerData;
        }
    }
}

// 메인 동작 루프: UDP 설정, Manus 호스트 연결, 실시간 데이터 송신 및 콘솔 출력 수행
void TeleopMasterClient::Run() {
    
    // [중요] 타겟 수신 PC (예: ROS2가 실행 중인 Ubuntu)의 실제 IP 및 Port로 변경하세요.
    if (!InitializeUDP("192.168.0.112", 12345)) {
        ClientLog::error("Failed to initialize UDP.");
        return;
    }

    // Manus Core와 연결될 때까지 주기적으로 재시도 (1초 간격)
    while (Connect() != ClientReturnCode::ClientReturnCode_Success) std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // 메인 데이터 스트리밍 루프 시작
    while (m_Running) {
        {
            // 백그라운드 콜백에서 데이터가 업데이트되는 것을 보호하기 위해 mutex 잠금
            std::lock_guard<std::mutex> lock(m_DataMutex);
            
            // 현재 프레임의 데이터를 타겟 IP로 전송
            SendUDPData();

            // 콘솔 화면을 지우고 현재 데이터 상태 출력
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


