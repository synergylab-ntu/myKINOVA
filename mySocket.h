#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <WinSock2.h>
#pragma comment(lib,"Ws2_32.lib")
#include <memory.h>
#include <string.h>

class mySocket
{

public:
	int myBind(SOCKET s, const struct sockaddr FAR* name, int namelen) {
		/*	WINSOCK_API_LINKAGE
				int
				WSAAPI
				bind(
					_In_ SOCKET s,
					_In_reads_bytes_(namelen) const struct sockaddr FAR * name,
					_In_ int namelen
				);
				*/
		int b = bind(s, name, namelen);
		return 0;
	}
	

};