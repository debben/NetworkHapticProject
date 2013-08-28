/*
UDP NETWORK UTILITIES
Don't forget to check the header file for the function descriptions.
*/
#include "udpnet.h"
#include "time.h"

SOCKET cs; //Socket handle
SOCKADDR_IN target; //Socket address information

SOCKET ss;
WSADATA w;


bool NetInit(){
        //Start up Winsock…
    WSADATA wsadata;

    int error = WSAStartup(0x0202, &wsadata);

    //Did something happen?
    if (error)
        return false;

    //Did we get the right Winsock version?
    if (wsadata.wVersion != 0x0202)
    {
        WSACleanup(); //Clean up Winsock
        return false;
    }
	return true;
}

bool SetupSocketToHost(int PortNo, char* IPAddress)
{
    //Fill out the information needed to initialize a socket…

    target.sin_family = AF_INET; // address family Internet
    target.sin_port = htons (PortNo); //Port to connect on
    target.sin_addr.s_addr = inet_addr (IPAddress); //Target IP
    if(target.sin_addr.s_addr == INADDR_NONE){
        printf("Invalid address\n");
    }

    cs = socket (AF_INET, SOCK_DGRAM, 0); //Create socket
    if (cs == INVALID_SOCKET)
    {
        printf("Couldn't create the socket\n");
        return false;
    }

        return true; //Success
}

//CLOSECONNECTION – shuts down the socket
void CloseConnection ()
{
    //Close the socket if it exists
    if (cs)
        closesocket(cs);
    if (ss)
        closesocket(ss);

    WSACleanup(); //Clean up Winsock
}

struct PacketsBufferStruct
{
	bool Active;
	clock_t time_to_send;
	char text[80];
};

const int PacketBufferSize = 10000, TimeDelay = 0;
PacketsBufferStruct PacketsBuffer[PacketBufferSize];
int NextBuffer = 0, NextToProcess = 0;

void SendToHost(char currentpos[]){
	//Store the packet into PacketsBuffer array, to be sent later (after the time delay)
    memset(PacketsBuffer[NextBuffer].text, 0, sizeof(PacketsBuffer[NextBuffer].text)); //Clear the buffer
    strcpy(PacketsBuffer[NextBuffer].text, currentpos);
	PacketsBuffer[NextBuffer].time_to_send = clock() + TimeDelay;
	PacketsBuffer[NextBuffer].Active = true;
	NextBuffer = NextBuffer == PacketBufferSize - 1 ? 0 : NextBuffer + 1;
	
	//Send the packets in queue which have waited long enough
	while(PacketsBuffer[NextToProcess].Active && PacketsBuffer[NextToProcess].time_to_send <= clock() && !(NextBuffer == NextToProcess))
	{
		PacketsBuffer[NextToProcess].Active = false;
		sendto(cs, PacketsBuffer[NextToProcess].text, sizeof(PacketsBuffer[NextToProcess].text), 0, (struct sockaddr *)&target, sizeof(struct sockaddr));
		NextToProcess = NextToProcess == PacketBufferSize - 1 ? 0 : NextToProcess + 1;
	}
}



int SetupSocketOnPort(int portno)
{

    SOCKADDR_IN addr; // The address structure for a TCP socket

    addr.sin_family = AF_INET;      // Address family
    addr.sin_port = htons (portno);   // Assign port to this socket

    addr.sin_addr.s_addr = htonl (INADDR_ANY);

    ss = socket (AF_INET, /*SOCK_STREAM*/SOCK_DGRAM, 0); // Create socket

    if (ss == INVALID_SOCKET)
    {
        printf("invalid socket\n");
        return false; //Don't continue if we couldn't create a //socket!!
    }

    if (bind(ss, (LPSOCKADDR)&addr, sizeof(addr)) == SOCKET_ERROR)
    {
       //We couldn't bind (this will happen if you try to bind to the same
       //socket more than once)
       printf("could not bind\n");
        return false;
    }

	printf("Socket prepared\n");

    return true;

}

void MyReceive(char* retval){
        char buffer[80];
        memset(buffer, 0, sizeof(buffer)); //Clear the buffer

        //Put the incoming text into our buffer
        recv (ss, buffer, sizeof(buffer)-1, 0);

        strcpy(retval, buffer);
}
