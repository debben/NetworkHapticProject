/*
UDP NETWORK UTILITIES
Don't forget to check the header file for the function descriptions.
*/
#include "udpnet.h"


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

#ifdef DON_CARES
void SendToHost(char currentpos[]){
    char text[80];
    memset(text, 0, sizeof(text)); //Clear the buffer
    strcpy(text, currentpos);

    sendto(cs, text, sizeof(text), 0, (struct sockaddr *)&target, sizeof(struct sockaddr));

}

#else

void SendToHost(HLdouble* ptr){
    char text[sizeof(HLdouble)*16];
    memset(text, 0, sizeof(text)); //Clear the buffer
    memcpy(text, ptr,sizeof(HLdouble)*16 );

    sendto(cs, text, sizeof(text), 0, (struct sockaddr *)&target, sizeof(struct sockaddr));

}

#endif


int SetupSocketOnPort(int portno)
{

    SOCKADDR_IN addr; // The address structure for a TCP socket

    addr.sin_family = AF_INET;      // Address family
    addr.sin_port = htons (portno);   // Assign port to this socket

    addr.sin_addr.s_addr = htonl (INADDR_ANY);

    ss = socket (AF_INET, /*SOCK_STREAM*/SOCK_DGRAM, 0); // Create socket

    if (ss == INVALID_SOCKET)
    {

		int error = WSAGetLastError();
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
        char buffer[16*sizeof(HLdouble)];
        memset(buffer, 0, sizeof(buffer)); //Clear the buffer

        //Put the incoming text into our buffer
        recv (ss, buffer, sizeof(buffer)-1, 0);

        strcpy(retval, buffer);
}
