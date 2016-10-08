#include "moons/cansend.h"

#include <termios.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include <ros/ros.h>
#include "std_msgs/String.h"

#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h> 
#include <pthread.h>
pthread_mutex_t canOperate_mutex = PTHREAD_MUTEX_INITIALIZER;

CanSendObj::CanSendObj()
{
		memset(ReceiveBuf,0,sizeof(ReceiveBuf) / sizeof(VCI_CAN_OBJ));
		#ifdef DBG_PRINTF
			printf("receivebuf is: %ld \n",sizeof(ReceiveBuf) / sizeof(VCI_CAN_OBJ));
		#endif

        if(VCI_OpenDevice(VCI_USBCAN2,0,0)!=1)
        {
            ROS_ERROR("[CAN] Open Device Error ...");
        }
        else
        {
        	ROS_INFO("[CAN] Open Device Success ...");
        }

        VCI_INIT_CONFIG config;
        config.AccCode=0;
        config.AccMask=0xffffffff;
        config.Filter=1;
        config.Mode=0;

        /*500 Kbps  0x00  0x1C*/
        /*1000 Kbps 0x00  0x14*/
        config.Timing0 = 0x00;
        config.Timing1 = 0x14;

        if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)
        {
            ROS_ERROR("[CAN] Init CAN Error ...");
        }
        if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
        {
            ROS_ERROR("[CAN] Start CAN Error ...");
        }

}

CanSendObj::~CanSendObj()
{
	VCI_CloseDevice(VCI_USBCAN2,0);
	VCI_CloseDevice(VCI_USBCAN2,1);
}

UNS8 CanSendObj::canSend(Message *m)
{
    UNS8 res;
	pthread_mutex_lock(&canOperate_mutex);
	res = canSend_driver(m);
	pthread_mutex_unlock(&canOperate_mutex);
    return res; // OK
}

UNS8 CanSendObj::canSend_driver(Message const *m)
{
    UNS8 data;
    VCI_CAN_OBJ peakMsg;
    peakMsg.ID=m-> cob_id;              			/* 11/29 bit code */
    peakMsg.ExternFlag = 0;                       /*CAN_INIT_TYPE_ST*/
    peakMsg.RemoteFlag = m->rtr;
    peakMsg.DataLen = m->len;
                      /* count of data bytes (0..8) */
    for(data = 0 ; data <  m->len; data ++)
      peakMsg.Data[data] = m->data[data];         	/* data bytes, up to 8 */

  #if defined DEBUG_MSG_CONSOLE_ON
    MSG("out : ");
    print_message(m);
  #endif

//    ROS_INFO("[SEND] %Xd-%Xd-%Xd-%Xd", peakMsg.Data[0], peakMsg.Data[1], peakMsg.Data[2], peakMsg.Data[3]);
    if(!(VCI_Transmit(VCI_USBCAN2, 0, 0, &peakMsg, 1) > 0)) {
      perror("canSend_driver (Peak_Linux) : error of writing.\n");
      return 1;
    }
    return 0;
}

void cansendCallback(const std_msgs::String::ConstPtr& msg)
{
   ROS_INFO("I heard: [%s]", msg->data.c_str());
}

/**
 * @param recResponse data received
 * @param sdoIndex    parameter id : velocity, counts
 * @param opIndex     controller id, p1 or p2
 * @return
 */
UNS8 CanSendObj::canReceiveUnpack(VCI_CAN_OBJ &recResponse, SDOIndex sdoIndex, int opIndex)
{
	pthread_mutex_lock(&canOperate_mutex);
	VCI_CAN_OBJ rec[2500];
	int reclen = 0;
	int bufferleng = sizeof(ReceiveBuf) / sizeof(VCI_CAN_OBJ);

	memset(ReceiveBuf, 0, bufferleng);
	reclen = VCI_Receive(VCI_USBCAN2, 0, 0, rec, 2500, 100);

	for(int i = 0; i < reclen; i++)
	{
		if((rec[i].Data[1] == sdoIndex.filed.indexL)
				&&(rec[i].Data[2] == sdoIndex.filed.indexH)
				&&((rec[i].ID & 0x0f) == opIndex))
		{
			recResponse = rec[i];
			pthread_mutex_unlock(&canOperate_mutex);
			return 0;
		}
	}
	pthread_mutex_unlock(&canOperate_mutex);
	return -1;
}

UNS8 CanSendObj::canReceive()
{
	VCI_CAN_OBJ rec[100];
	int reclen = 0;
	int bufferleng;
	bufferleng = sizeof(ReceiveBuf) / sizeof(VCI_CAN_OBJ);
	memset(ReceiveBuf,0,bufferleng);
	pthread_mutex_lock(&canOperate_mutex);
	reclen = VCI_Receive(VCI_USBCAN2,0,0,rec,100,100);
#ifdef DBG_PRINTF
	for(int j; j < reclen; j++)
	{
		for(int i = 0; i < 8;i++)
		{
			printf("canReceive response  %08x \n",rec[j].Data[i]);
		}
	}
#endif
	if(reclen < bufferleng)
	{
		memcpy(ReceiveBuf,rec,reclen);
	}
	pthread_mutex_unlock(&canOperate_mutex);
	return reclen;
}

UNS8 CanSendObj::canReceive_loop()
{
	VCI_CAN_OBJ rec[1000];
	int lLong = 0;
	int reclen = 0;
	int totalReclen = 0;
	while(1)
	{
		lLong++;
		usleep(1000);
		printf("this index:%d \n",lLong);
		pthread_mutex_lock(&canOperate_mutex);
		reclen = VCI_Receive(VCI_USBCAN2,0,0,rec,1000,1000);
		pthread_mutex_unlock(&canOperate_mutex);
		if(reclen > 0)
		{
			totalReclen += reclen;
			printf("IND:%d Receive: %08X",0,rec[reclen - 1].ID);
			for(int i = 0; i < rec[reclen -1].DataLen; i++)
			{
				printf(" %08X", rec[reclen - 1].Data[i]);

			}
			printf("\n");
		}
		else if(reclen == -1)
		{
			printf("device cannot opend \n");
		}
	}
}








