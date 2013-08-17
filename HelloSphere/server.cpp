
#include "server.h"

SOCKET s;
WSADATA w;

int SetupSocketOnPort(int portno)
{
    int error = WSAStartup (0x0202, &w);   // Fill in WSA info

    if (error)
    {
        printf("error\n");
        return false; //For some reason we couldn't start Winsock
    }

    if (w.wVersion != 0x0202) //Wrong Winsock version?
    {
        printf("version problem\n");
        WSACleanup ();
        return false;
    }

    SOCKADDR_IN addr; // The address structure for a TCP socket

    addr.sin_family = AF_INET;      // Address family
    addr.sin_port = htons (portno);   // Assign port to this socket

    addr.sin_addr.s_addr = htonl (INADDR_ANY);

    s = socket (AF_INET, /*SOCK_STREAM*/SOCK_DGRAM, 0); // Create socket

    if (s == INVALID_SOCKET)
    {
        printf("invalid socket\n");
        return false; //Don't continue if we couldn't create a //socket!!
    }

    if (bind(s, (LPSOCKADDR)&addr, sizeof(addr)) == SOCKET_ERROR)
    {
       //We couldn't bind (this will happen if you try to bind to the same
       //socket more than once)
       printf("could not bind\n");
        return false;
    }

	printf("Socket prepared");

    return true;

}

void myreceive(char* retval){
        char buffer[80];
        memset(buffer, 0, sizeof(buffer)); //Clear the buffer

		printf("before recv\n");
        //Put the incoming text into our buffer
        recv (s, buffer, sizeof(buffer)-1, 0);
		printf("after recv\n");

        strcpy(retval, buffer);
}

//CLOSECONNECTION – shuts down the socket
void CloseConnection ()
{
    //Close the socket if it exists
    if (s)
        closesocket(s);

    WSACleanup(); //Clean up Winsock
}
