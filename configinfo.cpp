#include "configinfo.h"

configinfo::configinfo()
{
    char sName[HOST_NAME_LENGTH+1];
    memset(sName,0,sizeof(sName));
    gethostname(sName,HOST_NAME_LENGTH);
    hostPtr = gethostbyname(sName);
}

configinfo::~configinfo()
{
    //dtor
}
void configinfo::readServerConfig()
{
    cout<<"Checking and reading server IP address.."<<endl;

    char serverConfigFile[50] = "serverConfig.txt";
    ifstream indata;
    indata.open(serverConfigFile);
    if(!indata)
    {
        cerr << "Error: file could not be opened" << endl;
        exit(1);
    }
    indata >> serverIPAddress;

    cout<<"Successfully retrieved the server information.."<<serverIPAddress<<endl;
    indata.close();
}

