#include "MoonsCmd.h"
#include <ros/ros.h>
#include <pthread.h>
#include "WheelCtr.h"
pthread_mutex_t MoonsCMD_mutex;

// Control Thread that updates the motors.
void *emergencyHandle_thread(void* ptr);

WheelCtr::WheelCtr()
{
	int rc1,iAckMotorFlag;
	moonscmdapp = new MoonsCMD;
	pthread_mutex_init(&MoonsCMD_mutex, NULL);
	period1=500000; //500ms
	iAckMotorFlag = ackMotormodes(1);
	if(iAckMotorFlag == -1)
	{
		ROS_ERROR("set motor1 modes failure.....\n");
	}
	else if(iAckMotorFlag == 0)
	{
		ROS_ERROR("set motor1 modes success.....\n");
	}
	iAckMotorFlag = ackMotormodes(2);
	if(iAckMotorFlag == -1)
	{
		ROS_ERROR("set motor2 modes failure.....\n");
	}
	else if(iAckMotorFlag == 0)
	{
		ROS_ERROR("set motor2 modes success.....\n");
	}
	if ((rc1 = pthread_create(&moonsCmdEmg_thread, NULL, emergencyHandle_thread, (void*)(this))))
	{
		printf("Thread2 creation failed: %d\n", rc1);
	}
}

WheelCtr::~WheelCtr() {}

void WheelCtr::getPositions(INTEGER32& g_w1, INTEGER32& g_w2)
{
	UNS32 t_w1 = 0;
	UNS32 t_w2 = 0;
	SDOIndex sdoIndex;
	static INTEGER32 previous_w1 = 0;
	static INTEGER32 previous_w2 = 0;
	sdoIndex.filed.indexL = 0x0A;
	sdoIndex.filed.indexH = 0x70;
	sdoIndex.filed.subIndex = 0;
	pthread_mutex_lock(&MoonsCMD_mutex);
	usleep(3000);
	/***********************************/
	this->moonscmdapp->GetCount(1);
	usleep(2000);
	if(this->moonscmdapp->unpackReadPara(sdoIndex,t_w1,1) == -1)
	{
		ROS_ERROR("[Chassis] P1 fetch data failure: %d, %d", sdoIndex.filed.indexH, sdoIndex.filed.indexL);
		t_w1 = previous_w1;

	}
	g_w1 = (INTEGER32)t_w1;
	/***********************************/
	usleep(1000);

	this->moonscmdapp->GetCount(2);
	usleep(2000);
	if(this->moonscmdapp->unpackReadPara(sdoIndex,t_w2,2) == -1)
	{
		ROS_ERROR("[Chassis] P2 fetch data failure: %d, %d", sdoIndex.filed.indexH, sdoIndex.filed.indexL);
		t_w2 = previous_w2;
	}
	g_w2 = (INTEGER32)t_w2;
	/***********************************/
	previous_w1 = g_w1;
	previous_w2 = g_w2;

	pthread_mutex_unlock(&MoonsCMD_mutex);
}

void WheelCtr::getVelocities(float& g_w1, float& g_w2)
{
	UNS32 t_w1 = 0;
	UNS32 t_w2 = 0;
	SDOIndex sdoIndex;
	sdoIndex.filed.indexL   = 0x09;
	sdoIndex.filed.indexH   = 0x70;
	sdoIndex.filed.subIndex = 0;

	pthread_mutex_lock(&MoonsCMD_mutex);
	/***********************************/
	this->moonscmdapp->GetVelocity(1);
	usleep(1000);
	this->moonscmdapp->unpackReadPara(sdoIndex,t_w1,1);
    // ((INTEGER16)t_w1 * 10000L * 60) / 240;
	g_w1 = (double)((INTEGER16)t_w1 * 0.25);
	/***********************************/
	this->moonscmdapp->GetVelocity(2);
	usleep(1000);
	this->moonscmdapp->unpackReadPara(sdoIndex,t_w2,2);
    // ((INTEGER16)t_w2 * 10000L * 60) / 240;
	g_w2 = (double)((INTEGER16)t_w2 * 0.25);
	/***********************************/
	pthread_mutex_unlock(&MoonsCMD_mutex);
}

void WheelCtr::setVelocities(float s_w1, float s_w2)
{

	pthread_mutex_lock(&MoonsCMD_mutex);
	usleep(2000);
	/***********************************/
	this->moonscmdapp->SetVelocity(s_w1,1);
	/***********************************/
	this->moonscmdapp->SetVelocity(s_w2,2);
	/***********************************/
	pthread_mutex_unlock(&MoonsCMD_mutex);
}

void WheelCtr::setStart()
{
	pthread_mutex_lock(&MoonsCMD_mutex);
	usleep(2000);
	this->moonscmdapp->SetStart(1);
	this->moonscmdapp->SetStart(2);
	pthread_mutex_unlock(&MoonsCMD_mutex);
}

int WheelCtr::ackMotormodes(int opIndex)
{
	SDOIndex sdoIndex;
	UNS32 t_w = 0;
	Tag_StatusWord tagStatusWord;
	tagStatusWord.status = 0;
	sdoIndex.filed.indexL   = 0x41;
	sdoIndex.filed.indexH   = 0x60;
	sdoIndex.filed.subIndex = 0;
	for(int i = 0; i < 3; i++)
	{
		pthread_mutex_lock(&MoonsCMD_mutex);
		usleep(2000);
		moonscmdapp->GetStatusword(opIndex);
		usleep(2000);
		moonscmdapp->unpackReadPara(sdoIndex,t_w,opIndex);
		tagStatusWord.status = t_w;
		pthread_mutex_unlock(&MoonsCMD_mutex);
		if(tagStatusWord.field.SwitchOn == 1 && tagStatusWord.field.ReadytoSwitchOn == 1)
		{
			return 0;
		}
		else
		{
			moonscmdapp->FaultReset(opIndex);
		}
	}
	return -1;
}


void *emergencyHandle_thread(void* ptr)
{
	static bool stoFlag1,stoFlag2;
	stoFlag1 = false;
	stoFlag2 = false;
	WheelCtr *pComm=(WheelCtr*) ptr;
	SDOIndex sdoIndex;
	UNS32 t_w1 = 0;
	UNS32 t_w2 = 0;
	Tag_dspstatus tagDspStatus1,tagDspStatus2;
	tagDspStatus1.status = 0;
	tagDspStatus2.status = 0;
	sdoIndex.filed.indexL   = 0x0B;
	sdoIndex.filed.indexH   = 0x70;
	sdoIndex.filed.subIndex = 0;
  while(true)
  {
	pthread_mutex_lock(&MoonsCMD_mutex);
	usleep(3000);
	pComm->moonscmdapp->GetDspStatus(1);
	usleep(2000);
	pComm->moonscmdapp->unpackReadPara(sdoIndex,t_w1,1);
	tagDspStatus1.status = t_w1;

	if(tagDspStatus1.field.DriveFault == 1)
	{
		stoFlag1 = true;
	}
	else
	{
		if(stoFlag1)
		{
			stoFlag1 = false;
			pComm->moonscmdapp->FaultReset(1);
			pComm->moonscmdapp->FaultReset(2);
		}
	}

	pthread_mutex_unlock(&MoonsCMD_mutex);

    usleep(pComm->period1);
  }
}






