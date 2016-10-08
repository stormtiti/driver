#ifndef MOONSCMD_H
#define MOONSCMD_H

#include "moons/applicfg.h"
#include "moons/cansend.h"

typedef union{
	struct{
		UNS16 MotorEnabled 		:1; //motor disabled is this bit = 0
		UNS16 Sampling			:1;
		UNS16 DriveFault		:1;
		UNS16 InPosition		:1;

		UNS16 Moving			:1;
		UNS16 Jogging			:1;
		UNS16 Stopping			:1;//stopping-in the process of stopping from a stop command
		UNS16 Waiting			:1;

		UNS16 Saving			:1;//
		UNS16 Alarmpresent		:1;//check alarm code
		UNS16 Homing			:1;//executing an SH command
		UNS16 WaitTime			:1;//executing a WT command

		UNS16 Wizardrunning 	:1;//timing wizard is running
		UNS16 Checkingencoder 	:1;//timing wizard is running
		UNS16 QProgramisruning 	:1;
		UNS16 initializing		:1;
	}field;
	UNS16 status;
}Tag_dspstatus;


typedef union{
	struct{
		UNS16 ReadytoSwitchOn 		:1; //motor disabled is this bit = 0
		UNS16 SwitchOn				:1;
		UNS16 OperationEnabled		:1;
		UNS16 Fault					:1;

		UNS16 VoltageDisabled		:1;
		UNS16 QuickStop				:1;
		UNS16 SwitchOnDisabled		:1;//stopping-in the process of stopping from a stop command
		UNS16 Warming				:1;

		UNS16 ManufacturerSpecific	:1;//
		UNS16 Remote				:1;//check alarm code
		UNS16 TargetReached			:1;//executing an SH command
		UNS16 InternalLimitActive	:1;//executing a WT command

		UNS16 Reserved 	:4;//timing wizard is running
	}field;
	UNS16 status;
}Tag_StatusWord;

class MoonsCMD
{
public:
	MoonsCMD();

    ~MoonsCMD(){};
    void getODentry(Message *m);
    void sendTest();
    void initSendData();
    //
    int SetVelocityMode(int opIndex);
    int FaultReset(int opIndex);
    int SetVelocity(float Spd,int opIndex);
    int SetStop(int opIndex);
    int SetStart(int opIndex);
    int GetVelocity(int opIndex);
    int GetCount(int opIndex);
    int GetDspStatus(int opIndex);
    int GetDspAlarm(int opIndex);
    int GetStatusword(int opIndex);
    int GetCurModes(int opIndex);
    //================================================
    //
    //=================================================
    int unpackReadPara(SDOIndex sdoIndex,UINT32& rdata,int opIndex);


    void SetAgvSpeed(float agvLineSpd,float agvRotateSpd);
    UNS8 const ReadyToSwitchOn[4] = {0x6,0x0,0x0,0x0};
    UNS8 const SwitchOn[4] = {0x7,0x0,0x0,0x0};
    UNS8 const StartMotion[4] = {0xF,0x0,0x0,0x0};
    UNS8 const OpEnableMotionHalted[4] = {0xF,0x1,0x0,0x0};
    UNS8 const SetToProfileVelocity[4] = {0x3,0x0,0x0,0x0};
    UNS8 const FaultResetCmd[4] = {0x80,0x00,0x00,0x00};
    SDODataType Motor_velocitySet;
    SDODataType Motor_controlword;
    SDODataType Motor_operationMode;
    SDODataType Motor_AccelerationSet;
    SDODataType Motor_DecelerationSet;
    SDODataType Motor_CountRead;
    SDODataType Motor_CurVelocityRead;
    SDODataType Motor_DspStatusCode;
    SDODataType Motor_Statusword;
    SDODataType Motor_CurModes;

    CanSendObj *cansendapp;

};
#endif // CANSEND_H
