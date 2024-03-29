#ifndef CIMAGECLIENT_H
#define CIMAGECLIENT_H


#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <fcntl.h>
#include <errno.h>
#include "CRawImage.h"
#include "CDecoder.h"
#include "CThread.h" 
#include <semaphore.h> 

typedef enum
{
	CMD_NONE = 0,
	CMD_IMAGE,
	CMD_COMPRESS,
	CMD_START_TRACK,
	CMD_STOP_TRACK,
	CMD_QUIT,
	CMD_NUMBER
}
TCommandType;

/**
@author Tom Krajnik
*/
class CImageClient:CThread
{
public:
    CImageClient(sem_t *im);
    ~CImageClient();

    int sendCommand(TCommandType type);
    int connectServer(const char * ip,const char* port);
    void run(CRawImage* image);
    int disconnectServer();
    CRawImage* image;

private:
    int DoExecute();
    int socketNumber;
    CDecoder* codec;
    SDL_Thread * thread;
    bool stop,stopped;
    sem_t *imageSem;
};

#endif
