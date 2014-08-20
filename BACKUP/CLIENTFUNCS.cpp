#include "CLIENTFUNCS.h"
#include <fstream>
#define DIRPATH "IMAGES\\"
using namespace std;

CLIENTFUNCS::CLIENTFUNCS()
{
    // Initialize Winsock.
    int iResult = WSAStartup( MAKEWORD(2,2), &wsaData );
    if ( iResult != NO_ERROR )
    {
        cout<<"Error at WSAStartup()..."<<endl;
        winLog<<"Error at WSAStartup()..."<<endl;
    }
    else
    {
        cout<<"Winsocket Initialised...."<<endl;
        winLog<<"Winsocket Initialised...."<<endl;
    }

    // Create a socket.
    clientSocket = socket( AF_INET, SOCK_STREAM, 0 );

    if ( clientSocket == INVALID_SOCKET )
    {
        printf( "Error at socket(): %d\n", WSAGetLastError() );
        winLog<<"Error at socket(): "<<WSAGetLastError()<<endl;
        WSACleanup();
        return;
    }

    clientBackup = clientSocket;
}

CLIENTFUNCS::~CLIENTFUNCS()
{
    WSACleanup();
}

void CLIENTFUNCS::connectServer(char *ip,int port)
{
    // Connect to a server.
    con.sin_family = AF_INET;
    con.sin_addr.s_addr = inet_addr( ip );
    con.sin_port = htons( port );

    if ( connect( clientSocket, (SOCKADDR*) &con, sizeof(con) ) == SOCKET_ERROR)
    {
        cout<<"Failed to connect..."<<endl;
        winLog<<"Failed to connect..."<<endl;
        WSACleanup();
        return;
    }
    else
    {
        cout<<"Connected to server..."<<endl;
        winLog<<"Connected to server..."<<endl;
    }
}


int CLIENTFUNCS::sendData(char *sendbuf)
{
    return send( clientSocket, sendbuf, strlen(sendbuf), 0 );
}


int CLIENTFUNCS::recvData(char *recvbuf,int size)
{
    int sz = recv( clientSocket, recvbuf, size, 0 );
    recvbuf[sz] = '\0';
    return sz;
}


void CLIENTFUNCS::closeConnection()
{
    closesocket(clientSocket);
    clientSocket = clientBackup;
    winLog<<"DisConnected to server..."<<endl;
}


void CLIENTFUNCS::fileReceive(char *filename)
{

    char rec[50] = "";
    char temp[1000] = "";
    recv( clientSocket, filename, 32, 0 );
    send( clientSocket, "OK", strlen("OK"), 0 );

    strcat(temp,DIRPATH);
    strcat(temp,filename);
    //cout<<"The filename is:"<<temp<<endl;
    FILE *fw = fopen(temp, "wb");

    int recs = recv( clientSocket, rec, 32, 0 );
    send( clientSocket, "OK", strlen("OK"), 0 );

    rec[recs]='\0';
    int size = atoi(rec);

    while(size > 0)
    {
        char buffer[1030];
        if(size>=1024)
        {
            recv( clientSocket, buffer, 1024, 0 );
            send( clientSocket, "OK", strlen("OK"), 0 );
            fwrite(buffer, 1024, 1, fw);
        }
        else
        {
            recv( clientSocket, buffer, size, 0 );
            send( clientSocket, "OK", strlen("OK"), 0 );
            buffer[size]='\0';
            fwrite(buffer, size, 1, fw);
        }
        size -= 1024;
    }
    fclose(fw);
}

void CLIENTFUNCS::fileSend(char *fpath)
{

    // Extract only filename from given path.
    char filename[50];
    int i=strlen(fpath);
    for(; i>0; i--)if(fpath[i-1]=='\\')break;
    for(int j=0; i<=(int)strlen(fpath); i++)filename[j++]=fpath[i];
    ////////////////////////////////////////

    ifstream myFile (fpath, ios::in|ios::binary|ios::ate);
    int size = (int)myFile.tellg();
    myFile.close();

    char filesize[10];
    itoa(size,filesize,10);


    send( clientSocket, filename, strlen(filename), 0 );
    char rec[32] = "";
    recv( clientSocket, rec, 32, 0 );

    send( clientSocket, filesize, strlen(filesize), 0 );
    recv( clientSocket, rec, 32, 0 );


    FILE *fr = fopen(fpath, "rb");

    while(size > 0)
    {
        char buffer[1030];

        if(size>=1024)
        {
            fread(buffer, 1024, 1, fr);
            send( clientSocket, buffer, 1024, 0 );
            recv( clientSocket, rec, 32, 0 );

        }
        else
        {
            fread(buffer, size, 1, fr);
            buffer[size]='\0';
            send( clientSocket, buffer, size, 0 );
            recv( clientSocket, rec, 32, 0 );
        }


        size -= 1024;

    }

    fclose(fr);

}
