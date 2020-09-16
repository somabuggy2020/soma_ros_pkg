#ifndef COMM_H
#define COMM_H

#include <iostream>
#include <string>
#include <math.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <error.h>
#include <errno.h>

#include <QUdpSocket>

#include "../Data/Data.h"

class CommNUC2;
class Clutch;
class Rotary;

class Communicate
{
public:
	Communicate();
	~Communicate();

	void recv(Data *data);
	void send(float lambda, float v);

private:
	CommNUC2 *commNUC2;
	Clutch *clutch;
	Rotary *rotary;
};

/*
Communication subclass for NUC2(Drive control)
*/
class CommNUC2
{
public:
	CommNUC2();
	~CommNUC2();

	int send(float lambda, float v);

private:
	int sock_send, sock_recv;
	struct sockaddr_in addr_send, addr_recv;
	std::string ip;
	int port;
};

/*
*/
class Clutch
{
public:
	Clutch();
	~Clutch();

	void _recv(Data *data);

private:
	int sock_send, sock_recv;
	struct sockaddr_in addr_send, addr_recv;
	std::string ip;
	int port_send, port_recv;

	QUdpSocket *qsock_recv;
};

/*
*/
class Rotary
{
public:
	Rotary();
	~Rotary();

	void _recv(Data *data);

private:
	int sock_send, sock_recv;
	struct sockaddr_in addr_send, addr_recv;
	std::string ip;
	int port_send, port_recv;

	QUdpSocket *qsock_recv;
};
#endif
