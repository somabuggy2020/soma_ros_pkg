#include "soma_ros/Communicate/Communicate.h"

Communicate::Communicate()
{
    ROS_INFO("Create communicate instance");
 
    commNUC2 = new CommNUC2();
    clutch = new Clutch();
    rotary = new Rotary();
}

Communicate::~Communicate()
{
}

void Communicate::recv(Data *data)
{
//    clutch->_recv(data);
//    rotary->_recv(data);
//    data->Ut.lambda = data->Uin.lambda;
    return;
}

void Communicate::send(float lambda, float v)
{
        commNUC2->send(lambda, v);
    return;
}

//
//
//
CommNUC2::CommNUC2()
{
        ip = "192.168.1.12";
        port = 7001;

        sock_send = socket(AF_INET, SOCK_DGRAM, 0);
        addr_send.sin_family = AF_INET;
        addr_send.sin_addr.s_addr = inet_addr(ip.c_str());
        addr_send.sin_port = htons(port);
}

CommNUC2::~CommNUC2(){
}

int CommNUC2::send(float lambda, float v)
{
        //Definition send data
        struct Send_t
        {
            int mode;
            float steering;
            float s;
            float isRotReset;
            float isWeeding;
        } send;

        if (v > 0.0) send.mode = 1; //Foward
        else if (v > 0.0) send.mode = 2; //Backward
        else send.mode = 3; //Stop

        send.steering = lambda / M_PI * 180.0;
        send.s = 1.0;
        send.isRotReset = false;
        send.isWeeding = false;

        int ret = sendto(sock_send,
                         (char *)&send, sizeof(Send_t), 0,
                         (struct sockaddr *)&addr_send, sizeof(addr_send));

        return 0;
}


//
//
//
Clutch::Clutch()
{
        ip = "192.168.1.79";
        // port_send = 22345;
        port_recv = 12345;

	qsock_recv = new QUdpSocket();
	qsock_recv->bind(12345);

        // sock_send = socket(AF_INET, SOCK_DGRAM, 0);
        // addr_send.sin_family = AF_INET;
        // addr_send.sin_addr.s_addr = inet_addr(ip.c_str());
        // addr_send.sin_port = htons(port_send);

/*        sock_recv = socket(AF_INET, SOCK_DGRAM, 0);
        addr_recv.sin_family = AF_INET;
        addr_recv.sin_addr.s_addr = inet_addr("0.0.0.0");
        addr_recv.sin_port = htons(port_send);
        bind(sock_recv, (const struct sockaddr *)&addr_recv, sizeof(addr_recv));
*/
};

Clutch::~Clutch(){
}

void Clutch::_recv(Data *data)
{
	int d = 0;
	if(qsock_recv->hasPendingDatagrams()){
		while(qsock_recv->pendingDatagramSize() > 0){
			qsock_recv->readDatagram((char*)&d, sizeof(int));
		}
		// ROS_INFO("Clutch Recv!:%d",d);
		data->clutch = d;
	}
	return;
}

//
//
//
Rotary::Rotary()
{
        ip = "192.168.1.79";
        port_send = 22346;
        port_recv = 12346;

	qsock_recv = new QUdpSocket();
	qsock_recv->bind(12346);


        // sock_send = socket(AF_INET, SOCK_DGRAM, 0);
        // addr_send.sin_family = AF_INET;
        // addr_send.sin_addr.s_addr = inet_addr(ip.c_str());
        // addr_send.sin_port = htons(port_send);

//        sock_recv = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
/*
	if(sock_recv < 0){
		ROS_WARN("recieve socket error");
		exit(1);
	}

        addr_recv.sin_family = AF_INET;
        addr_recv.sin_addr.s_addr = INADDR_ANY;
        addr_recv.sin_port = htons(port_recv);
        
        int len = 0;
	socklen_t optlen = sizeof(len);
        if(getsockopt(sock_recv, SOL_SOCKET, SO_RCVBUF, &len, &optlen) < 0){
            perror("getsockopt");
        }
	else{
		ROS_INFO("1:>%d", len);
	}

        struct Recv_t
        {
            unsigned long pulse_count;
            double velocity;
        };

       	int s1 = sizeof(long)+sizeof(double);
	int optlen = sizeof(s1);
        setsockopt(sock_recv, SOL_SOCKET, SO_RCVBUF, &s1, optlen);

	optlen = sizeof(len);
        if(getsockopt(sock_recv, SOL_SOCKET, SO_RCVBUF,&len, &optlen) < 0){
            perror("getsockopt");
        }
	else{
		ROS_INFO("2:>%d",len);
	}

	int on = 1;
	if(setsockopt(sock_recv, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) < 0){
		perror("set reuse");
	}
        
	int ret = bind(sock_recv, (const struct sockaddr *)&addr_recv, sizeof(addr_recv));
	if(ret < 0){
		perror("bind");
	}

        int val = 1;
        ioctl(sock_recv, FIONBIO, &val);
*/


}

Rotary::~Rotary()
{
}

void Rotary::_recv(Data *data)
{
//	ROS_INFO("rot recv");
        struct Recv_t
        {
            unsigned long pulse_count;
            double velocity;
        } d;
	d.pulse_count = 0;
	d.velocity = 0.0;

	if(qsock_recv->hasPendingDatagrams()){
		while(qsock_recv->pendingDatagramSize() > 0){
			qsock_recv->readDatagram((char*)&d, sizeof(Recv_t));
		}
//		ROS_INFO("Recv!");
//		ROS_INFO("%d / %f", d.pulse_count, d.velocity);

		// if(data->clutch == 1) data->Ut.v = d.velocity;
		// else if(data->clutch == 2) data->Ut.v = -d.velocity;
		// else if(data->clutch == 3) data->Ut.v = 0.0;
	}

	//struct sockaddr_in from;
	//socklen_t sin_size;

	//int n = recv(sock_recv, (char*)&d, sizeof(Recv_t), 0);
    //	int n =	recvfrom(sock_recv, (void*)&d, sizeof(Recv_t), 0,
    //			(struct sockaddr*)&from, &sin_size);

	/*if(n < 0){
		perror("socket");
	}
	if(errno == EAGAIN){
		perror("EAGAIN");
	}
	else{
		ROS_INFO("RECIEVE!!! %f[m/s]", d.velocity);
		data->vt_rot = d.velocity;
	}*/
}

