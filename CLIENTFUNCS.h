#ifndef CLIENTFUNCS_H
#define CLIENTFUNCS_H

#include "winsock2.h"
#include <stdio.h>
#include <conio.h>
#include <iostream>
#include <fstream>
#include "myLog.h"

using namespace std;

class CLIENTFUNCS
{
public:
    CLIENTFUNCS();
    virtual ~CLIENTFUNCS();

    WSADATA wsaData;
    SOCKET clientSocket;
    SOCKET clientBackup;
    sockaddr_in con;
    SOCKET AcceptSocket;

    void connectServer(char*,int);
    int sendData(char*);
    int recvData(char*,int);
    void fileSend(char*);
    void fileReceive(char*);
    void closeConnection();
};

#endif // CLIENTFUNCS_H
