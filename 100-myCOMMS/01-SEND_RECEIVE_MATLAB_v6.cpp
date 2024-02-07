#define WIN32_LEAN_AND_MEAN

#include <winsock2.h>
#include <Ws2tcpip.h>
#include <stdio.h>

// Link with ws2_32.lib
#pragma comment(lib, "Ws2_32.lib")

#define DEFAULT_BUFLEN 512
#define SEND_PORT 27015
#define RECV_PORT 27016

// Include for RECV
#ifndef UNICODE
#define UNICODE
#endif


#include <iostream>
using namespace std;

#include <sys/types.h>

#include <rl/math/Transform.h>
#include <rl/math/Unit.h>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/Dynamic.h>
#include <rl/mdl/Model.h>
#include <rl/mdl/UrdfFactory.h>
#include <rl/mdl/XmlFactory.h>
#include <boost/lexical_cast.hpp>


int __cdecl main() {

    const std::string robot_model = "D:/myKinova_v2/Robot/GEN3_GRIPPER_2024.urdf";
    rl::mdl::UrdfFactory factory;
    std::shared_ptr<rl::mdl::Model> model(factory.create(robot_model));

    rl::mdl::Dynamic* dynamics = dynamic_cast<rl::mdl::Dynamic*>(model.get());

    rl::math::Vector UDP_q(dynamics->getDof()+1);

    //----------------------
    // Declare and initialize variables - SEND
    WSADATA wsaData;
    int iResult;

    SOCKET ConnectSocket = INVALID_SOCKET;
    struct sockaddr_in clientService;
    //int SenderAddrSize = sizeof(clientService);

    char* sendbuf = "this is a test";
    char recvbuf[DEFAULT_BUFLEN];
    int recvbuflen = DEFAULT_BUFLEN;

    // Declare and initialize variables - RECV
    int iResult2 = 0;

    SOCKET RecvSocket;
    struct sockaddr_in RecvAddr;

    unsigned short Port2 = 27016;

    char RecvBuf[1024];
    int BufLen = 1024;

    struct sockaddr_in SenderAddr;
    int SenderAddrSize = sizeof(SenderAddr);

    //----------------------
    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult != NO_ERROR) {
        printf("WSAStartup failed: %d\n", iResult);
        return 1;
    }

    //----------------------
    // Create a SOCKET for connecting to server - SEND
    ConnectSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (ConnectSocket == INVALID_SOCKET) {
        printf("Error at socket(): %ld\n", WSAGetLastError());
        WSACleanup();
        return 1;
    }

    // Create a receiver socket to receive datagrams - RECV
    RecvSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    unsigned long ul = 1;
    int           nRet;
    nRet = ioctlsocket(RecvSocket, FIONBIO, (unsigned long*)&ul);

    //struct timeval read_timeout;
    //read_timeout.tv_sec = 0;
    //read_timeout.tv_usec = 10;
    //setsockopt(RecvSocket, SOL_SOCKET, SO_RCVTIMEO, (const char*)&read_timeout, sizeof read_timeout);
    int ret, iVal = 0;
    unsigned int  sz = sizeof(iVal);
    iVal = 1; // if you set this too low, the recvsocket will be impatient and might not 
    ret = setsockopt(RecvSocket, SOL_SOCKET, SO_RCVTIMEO, (char*)&iVal, sz);

    if (RecvSocket == INVALID_SOCKET) {
        printf("socket failed with error %d\n", WSAGetLastError());
        return 1;
    }

    //----------------------
    // The sockaddr_in structure specifies the address family,
    // IP address, and port of the server to be connected to. - SEND
    clientService.sin_family = AF_INET;
    clientService.sin_addr.s_addr = inet_addr("127.0.0.1");
    clientService.sin_port = htons(SEND_PORT);

    // Bind the socket to any address and the specified port - RECV
    RecvAddr.sin_family = AF_INET;
    RecvAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
    RecvAddr.sin_port = htons(RECV_PORT);

    iResult2 = ::bind(RecvSocket, (SOCKADDR*)&RecvAddr, sizeof(RecvAddr));
    if (iResult2 != 0) {
        printf("bind failed with error %d\n", WSAGetLastError());
        return 1;
    }

    //----------------------
    // Connect to server.
    iResult = connect(ConnectSocket, (SOCKADDR*)&clientService, sizeof(clientService));
    if (iResult == SOCKET_ERROR) {
        closesocket(ConnectSocket);
        printf("Unable to connect to server: %ld\n", WSAGetLastError());
        WSACleanup();
        return 1;
    }

    // Send an initial buffer
    iResult = send(ConnectSocket, sendbuf, (int)strlen(sendbuf), 0);
    if (iResult == SOCKET_ERROR) {
        printf("send failed: %d\n", WSAGetLastError());
        closesocket(ConnectSocket);
        WSACleanup();
        return 1;
    }

    printf("Bytes Sent: %ld\n", iResult);

    int delimiter_idx = 0;

    // Receive until the peer closes the connection
    do {
        printf("Sending datagrams...\n");

        iResult = send(ConnectSocket, sendbuf, (int)strlen(sendbuf), 0);
        printf("datagrams sent...\n");
        // Call the recvfrom function to receive datagrams
// on the bound socket. - RECV
        //wprintf(L"Receiving datagrams...\n");
        //iResult2 = recvfrom(RecvSocket,
        //    RecvBuf, BufLen, 0, (SOCKADDR*)&SenderAddr, &SenderAddrSize);
        iResult2 = recv(RecvSocket, RecvBuf, BufLen, 0);
        printf("Received datagrams...\n");

        if (iResult2 > 0) // print recvbuffer ONLY if something was received
        {
            std::cout << RecvBuf << std::endl;
            string myMATLAB_DATA(RecvBuf);
            std::string delimiter = "||";
            size_t pos = 0;
            std::string token;
            delimiter_idx = 0;
            while ((pos = myMATLAB_DATA.find(delimiter)) != std::string::npos) {
                token = myMATLAB_DATA.substr(0, pos);
                //std::cout << token << std::endl;
                
                float num_float = std::stof(token);

                UDP_q[delimiter_idx] = num_float;

                myMATLAB_DATA.erase(0, pos + delimiter.length());
                delimiter_idx++;
                if (delimiter_idx > 6)
                {
                    break;
                }
            }

            std::cout << "q1 =" << UDP_q[0] << " q2 =" << UDP_q[1] << " q3 =" << UDP_q[2] << " q4 =" << UDP_q[3] << " q5 =" << UDP_q[4] << " q6 =" << UDP_q[5] << " q7 =" << UDP_q[6] << std::endl;
            
            //std::string token = myMATLAB_DATA.substr(0, myMATLAB_DATA.find(delimiter)); // 
            //std::cout << token << std::endl;
            //UDP_q
        }

    } while (iResult > 0);

    // cleanup - SEND
    closesocket(ConnectSocket);

    // Close the socket when finished receiving datagrams - RECV
    wprintf(L"Finished receiving. Closing socket.\n");
    iResult2 = closesocket(RecvSocket);


    WSACleanup();

    return 0;
}