#include <windows.h>
#include <winsock.h>
#include <stdio.h>
#include <conio.h>
#include <HL/hl.h> //ugly, but what the fuck, we'll just leave this here for now. We'll fix it later.
/*
UDP NETWORK UTILITIES
Header containing functions for a SINGLE incoming and a SINGLE outgoing data sockets (UDP);
*/

//Must be called in the beggining of the program to initialize network library
bool NetInit();

//creates server (receive) socket
int SetupSocketOnPort(int portno);
//receives data as string
void MyReceive(char* str);
//creates client (send) socket
bool SetupSocketToHost(int, char[]);
//sends data
#ifdef DON_CARES
void SendToHost(char*);
#else
void SendToHost(HLdouble*);
#endif
//must be called in the end of the program, or after finishing using network functions
void CloseConnection();
