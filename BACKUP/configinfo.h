#ifndef CONFIGINFO_H
#define CONFIGINFO_H

#include "string.h"
#include "stdlib.h"
#include <iostream>
#include <fstream>
#include <winsock2.h>

using namespace std;

enum hostType {NAME, ADDRESS};
const int HOST_NAME_LENGTH = 64;
class configinfo
{
private:
    struct hostent *hostPtr;
public:
    configinfo();
    char serverIPAddress[100] ;
    virtual ~configinfo();
    void readServerConfig();

    char* getHostIPAddress()
    {
        struct in_addr *addr_ptr;
        addr_ptr = (struct in_addr *)*hostPtr->h_addr_list;
        return inet_ntoa(*addr_ptr);
    }

    char* getHostName()
    {
        return hostPtr->h_name;
    }
};

#endif // CONFIGINFO_H
