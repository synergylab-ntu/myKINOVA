
// Includes for UDP - start
#define WIN32_LEAN_AND_MEAN

#include <winsock2.h>
#include <Ws2tcpip.h>
#include <stdio.h>

// Link with ws2_32.lib
#pragma comment(lib, "Ws2_32.lib")

#define DEFAULT_BUFLEN 512

// Include for RECV
#ifndef UNICODE
#define UNICODE
#endif

#include <iostream>

#include <sys/types.h>
// Includes for UDP - end


class myKINOVA_UDP
{
private:



public:

    // UDP Receive global variables
    std::string UDPsendbuf;
    const char* sendbuf3;
    float num_TOSEND[7] = { 0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f };
    int num_TOSEND_idx;
    std::string delimiter = "||";
    float num_float;
    size_t STRINGpos;
    std::string token;

    u_short SEND_PORT;
    u_short RECV_PORT;
    const char* SEND_IP_ADDRESS;
    const char* RECV_IP_ADDRESS;
    int CTRL_MODE = 0;

    // Declare and initialize variables - SEND
    WSADATA wsaData;
    int iResult;

    SOCKET ConnectSocket = INVALID_SOCKET;
    struct sockaddr_in clientService;

    char* sendbuf = "this is a test";
    char recvbuf[DEFAULT_BUFLEN];
    int recvbuflen = DEFAULT_BUFLEN;

    // Declare and initialize variables - RECV
    int iResult2 = 0;

    SOCKET RecvSocket;
    struct sockaddr_in RecvAddr;

    char RecvBuf[1024];
    int BufLen = 1024;

    struct sockaddr_in SenderAddr;
    int SenderAddrSize = sizeof(SenderAddr);

    int ret, iVal = 1;
    unsigned int  sz = sizeof(iVal);

    float UDP_q[7] = { 0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
    float UDP_gripper = 0.0f;
    float UDP_tau[7] = { 0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};

    void setup_UDP()
    {
        // define Winsock2 object
        WSADATA wsaData;

        // Initialise SEND UDP variables
        struct sockaddr_in clientService;

        // Initialise RECV UDP variabls
        SOCKET RecvSocket;
        struct sockaddr_in RecvAddr;

        struct sockaddr_in SenderAddr;
        int SenderAddrSize = sizeof(SenderAddr);

        // initialise winsock
        initialize_Winsock();
    }

    int initialize_Winsock()
    {
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

        // Make port non-blocking - VERY IMPORTANT
        unsigned long ul = 1;
        int           nRet;
        nRet = ioctlsocket(RecvSocket, FIONBIO, (unsigned long*)&ul);

        iVal = 1; // if you set this too low, the recvsocket will be impatient and might not 
        ret = setsockopt(RecvSocket, SOL_SOCKET, SO_RCVTIMEO, (char*)&iVal, sz);

        if (RecvSocket == INVALID_SOCKET) {
            wprintf(L"socket failed with error %d\n", WSAGetLastError());
            return 1;
        }

        //----------------------
        // The sockaddr_in structure specifies the address family,
        // IP address, and port of the server to be connected to. - SEND
        clientService.sin_family = AF_INET;
        clientService.sin_addr.s_addr = inet_addr(SEND_IP_ADDRESS);
        clientService.sin_port = htons(SEND_PORT);

        // Bind the socket to any address and the specified port - RECV
        RecvAddr.sin_family = AF_INET;
        RecvAddr.sin_addr.s_addr = inet_addr(RECV_IP_ADDRESS);
        RecvAddr.sin_port = htons(RECV_PORT);

        iResult2 = ::bind(RecvSocket, (SOCKADDR*)&RecvAddr, sizeof(RecvAddr));
        if (iResult2 != 0) {
            wprintf(L"bind failed with error %d\n", WSAGetLastError());
            return 1;
        }

        printf("Connect instead of bind was run.\n");
        if (iResult2 == -1) {
            printf("Connect instead of bind failed.\n");
            exit(2);
        }

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
    }

    void UDP_send_recv_v4(float* input_pos)
    {
        int delimiter_idx = 0;

        UDPsendbuf = "";
        UDPsendbuf.append("q_start"); // string to send

        for (num_TOSEND_idx = 0; num_TOSEND_idx < 7; num_TOSEND_idx++) {
            num_TOSEND[num_TOSEND_idx] = input_pos[num_TOSEND_idx];
            UDPsendbuf.append(std::to_string(num_TOSEND[num_TOSEND_idx]));
            UDPsendbuf.append("||");

        }
        UDPsendbuf.append("q_end");

        // UDP part - start
        //wprintf(L"Sending datagrams...\n");

        sendbuf3 = UDPsendbuf.c_str();

        iResult = send(ConnectSocket, sendbuf3, (int)strlen(sendbuf3), 0);
        wprintf(L"datagrams sent...\n");
        iResult2 = recvfrom(RecvSocket,
            RecvBuf, BufLen, 0, (SOCKADDR*)&SenderAddr, &SenderAddrSize);

        //iResult2 = recv(RecvSocket, RecvBuf, BufLen, 0);

        //if (iResult2 > 0) // print recvbuffer ONLY if something was received
        //{
        //    wprintf(L"Received datagrams...\n");

        //    //std::cout << RecvBuf << std::endl;
        //    std::string myMATLAB_DATA(RecvBuf);

        //    STRINGpos = 0;
        //    token = "";
        //    delimiter_idx = 0;
        //    while ((STRINGpos = myMATLAB_DATA.find(delimiter)) != std::string::npos) {
        //        token = myMATLAB_DATA.substr(0, STRINGpos);
        //        num_float = std::stof(token);
        //        

        //        myMATLAB_DATA.erase(0, STRINGpos + delimiter.length());
        //        delimiter_idx++;
        //        if (delimiter_idx <= 6)
        //        {
        //            UDP_q[delimiter_idx] = num_float;
        //        }
        //        if (CTRL_MODE == 0)
        //        {
        //            if (delimiter_idx > 6)
        //            {
        //                break;
        //            }
        //        }
        //        if (CTRL_MODE >0) {
        //            if (delimiter_idx > 6 && delimiter_idx <= 13)
        //            {
        //                UDP_tau[delimiter_idx] = num_float;
        //            }
        //            if (delimiter_idx > 13)
        //            {
        //                break;
        //            }
        //        }
        //        
        //    }
        //    std::cout << "UDPq1 =" << UDP_q[0] << " UDPq2 =" << UDP_q[1] << " UDPq3 =" << UDP_q[2] << " UDPq4 =" << UDP_q[3] << " UDPq5 =" << UDP_q[4] << " UDPq6 =" << UDP_q[5] << " UDPq7 =" << UDP_q[6] << std::endl;
        //    std::cout << "UDPtau1 =" << UDP_tau[0] << " UDPtau2 =" << UDP_tau[1] << " UDPtau3 =" << UDP_tau[2] << " UDPtau4 =" << UDP_tau[3] << " UDPtau5 =" << UDP_tau[4] << " UDPtau6 =" << UDP_tau[5] << " UDPtau7 =" << UDP_tau[6] << std::endl;
        //}
        if (iResult2 > 0) // print recvbuffer ONLY if something was received
        {
            //wprintf(L"Received datagrams...\n");

            //std::cout << RecvBuf << std::endl;
            std::string myMATLAB_DATA(RecvBuf);

            STRINGpos = 0;
            token = "";
            delimiter_idx = 0;
            while ((STRINGpos = myMATLAB_DATA.find(delimiter)) != std::string::npos) {
                token = myMATLAB_DATA.substr(0, STRINGpos);
                num_float = std::stof(token);
                if (CTRL_MODE == 0 && delimiter_idx <= 6) {
                    UDP_q[delimiter_idx] = num_float;
                }

                if ((CTRL_MODE == 1 || CTRL_MODE ==2) && delimiter_idx <= 13) {
                    UDP_tau[delimiter_idx-7] = num_float;
                }

                if (CTRL_MODE == 5 && delimiter_idx <= 6) {
                    UDP_q[delimiter_idx] = num_float;
                }

                if (CTRL_MODE == 5 && delimiter_idx == 7) {
                    UDP_gripper = num_float;
                }

                myMATLAB_DATA.erase(0, STRINGpos + delimiter.length());
                delimiter_idx++;
                    if (CTRL_MODE == 0 && delimiter_idx > 6)
                    {
                        break;
                    }
                    if ((CTRL_MODE == 1 || CTRL_MODE == 2) && delimiter_idx > 13)
                    {
                        break;
                    }
                    if (CTRL_MODE == 5 && delimiter_idx > 7)
                    {
                        break;
                    }
            }
            std::cout << "q1 =" << UDP_q[0] << " q2 =" << UDP_q[1] << " q3 =" << UDP_q[2] << " q4 =" << UDP_q[3] << " q5 =" << UDP_q[4] << " q6 =" << UDP_q[5] << " q7 =" << UDP_q[6] << std::endl;
            std::cout << "tau1 =" << UDP_tau[0] << " tau2 =" << UDP_tau[1] << " tau3 =" << UDP_tau[2] << " tau4 =" << UDP_tau[3] << " tau5 =" << UDP_tau[4] << " tau6 =" << UDP_tau[5] << " tau7 =" << UDP_tau[6] << std::endl;
            std::cout << "gripper value is : " << UDP_gripper << std::endl;
        }
        else
        {
            wprintf(L"Received no datagrams...\n");
            // if no UDP communication occurred, set UDP_q should not change.
            
        }

        // UDP part - end
    }

    //	// UDP close - start
    //	// cleanup - SEND
    void cleanup() {
        // SEND socket close
        closesocket(ConnectSocket);

        //RECV socket close
        iResult2 = closesocket(RecvSocket);
        WSACleanup();
    }


    myKINOVA_UDP(){// default constructor
    }

    myKINOVA_UDP(int CTRL_MODE_input, u_short SEND_PORT_input, u_short RECV_PORT_input, const char* SEND_IP_input, const char* RECV_IP_input)
    {
        CTRL_MODE = CTRL_MODE_input;
        SEND_PORT = SEND_PORT_input;
        RECV_PORT = RECV_PORT_input;
        SEND_IP_ADDRESS = SEND_IP_input;
        RECV_IP_ADDRESS = RECV_IP_input;
    }


};