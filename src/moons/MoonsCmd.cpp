#include "moons/cansend.h"
#include "moons/MoonsCmd.h"
#include <termios.h>
#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include <ros/ros.h>

namespace {
float fspdMax = 3500;
float fspdMin = -3500;
}

MoonsCMD::MoonsCMD() {

	initSendData();
	cansendapp = new CanSendObj();
	usleep(5000);
	SetVelocityMode(1);
	SetVelocityMode(2);
}

void MoonsCMD::initSendData() {
	//volitile
	Motor_velocitySet.filed.cmd = 0x23;
	Motor_velocitySet.filed.indexL = 0xFF;
	Motor_velocitySet.filed.indexH = 0x60;
	Motor_velocitySet.filed.subIndex = 0x00;
	Motor_velocitySet.filed.data0 = 0;
	Motor_velocitySet.filed.data1 = 0;
	Motor_velocitySet.filed.data2 = 0;
	Motor_velocitySet.filed.data3 = 0;

	//controlword
	Motor_controlword.filed.cmd = 0x2B;
	Motor_controlword.filed.indexL = 0x40;
	Motor_controlword.filed.indexH = 0x60;
	Motor_controlword.filed.subIndex = 0x00;
	Motor_controlword.filed.data0 = 0;
	Motor_controlword.filed.data1 = 0;
	Motor_controlword.filed.data2 = 0;
	Motor_controlword.filed.data3 = 0;

	//operationMode
	Motor_operationMode.filed.cmd = 0x2F;
	Motor_operationMode.filed.indexL = 0x60;
	Motor_operationMode.filed.indexH = 0x60;
	Motor_operationMode.filed.subIndex = 0x00;
	Motor_operationMode.filed.data0 = 0;
	Motor_operationMode.filed.data1 = 0;
	Motor_operationMode.filed.data2 = 0;
	Motor_operationMode.filed.data3 = 0;

	//Acceleration
	Motor_AccelerationSet.filed.cmd = 0x23;
	Motor_AccelerationSet.filed.indexL = 0x83;
	Motor_AccelerationSet.filed.indexH = 0x60;
	Motor_AccelerationSet.filed.subIndex = 0x00;
	Motor_AccelerationSet.filed.data0 = 0;
	Motor_AccelerationSet.filed.data1 = 0;
	Motor_AccelerationSet.filed.data2 = 0;
	Motor_AccelerationSet.filed.data3 = 0;

	//Deceleration
	Motor_DecelerationSet.filed.cmd = 0x23;
	Motor_DecelerationSet.filed.indexL = 0x84;
	Motor_DecelerationSet.filed.indexH = 0x60;
	Motor_DecelerationSet.filed.subIndex = 0x00;
	Motor_DecelerationSet.filed.data0 = 0;
	Motor_DecelerationSet.filed.data1 = 0;
	Motor_DecelerationSet.filed.data2 = 0;
	Motor_DecelerationSet.filed.data3 = 0;

	//Count
	Motor_CountRead.filed.cmd = 0x40;
	Motor_CountRead.filed.indexL = 0x0A;
	Motor_CountRead.filed.indexH = 0x70;
	Motor_CountRead.filed.subIndex = 0x00;
	Motor_CountRead.filed.data0 = 0;
	Motor_CountRead.filed.data1 = 0;
	Motor_CountRead.filed.data2 = 0;
	Motor_CountRead.filed.data3 = 0;

	//CurVelocity
	Motor_CurVelocityRead.filed.cmd = 0x40;
	Motor_CurVelocityRead.filed.indexL = 0x09;
	Motor_CurVelocityRead.filed.indexH = 0x70;
	Motor_CurVelocityRead.filed.subIndex = 0x00;
	Motor_CurVelocityRead.filed.data0 = 0;
	Motor_CurVelocityRead.filed.data1 = 0;
	Motor_CurVelocityRead.filed.data2 = 0;
	Motor_CurVelocityRead.filed.data3 = 0;

	//DSP_status_code
	Motor_DspStatusCode.filed.cmd = 0x40;
	Motor_DspStatusCode.filed.indexL = 0x0B;
	Motor_DspStatusCode.filed.indexH = 0x70;
	Motor_DspStatusCode.filed.subIndex = 0x00;
	Motor_DspStatusCode.filed.data0 = 0;
	Motor_DspStatusCode.filed.data1 = 0;
	Motor_DspStatusCode.filed.data2 = 0;
	Motor_DspStatusCode.filed.data3 = 0;

	//status_word
	Motor_Statusword.filed.cmd = 0x40;
	Motor_Statusword.filed.indexL = 0x41;
	Motor_Statusword.filed.indexH = 0x60;
	Motor_Statusword.filed.subIndex = 0x00;
	Motor_Statusword.filed.data0 = 0;
	Motor_Statusword.filed.data1 = 0;
	Motor_Statusword.filed.data2 = 0;
	Motor_Statusword.filed.data3 = 0;


	//Modes
	Motor_CurModes.filed.cmd = 0x40;
	Motor_CurModes.filed.indexL = 0x61;
	Motor_CurModes.filed.indexH = 0x60;
	Motor_CurModes.filed.subIndex = 0x00;
	Motor_CurModes.filed.data0 = 0;
	Motor_CurModes.filed.data1 = 0;
	Motor_CurModes.filed.data2 = 0;
	Motor_CurModes.filed.data3 = 0;
}

int MoonsCMD::FaultReset(int opIndex) {
	Message mSendMsg;
	int temp, tempCodID;
	//Ready to Switch on
	if ((opIndex > 0) && (opIndex < 9)) {
		tempCodID = 0x600 + opIndex;
	} else {
		return -1;
	}
	usleep(5000);
	mSendMsg.cob_id = tempCodID;
	mSendMsg.len = 8;
	mSendMsg.rtr = 0;
	Motor_controlword.filed.data0 = FaultResetCmd[0];
	Motor_controlword.filed.data1 = FaultResetCmd[1];
	Motor_controlword.filed.data2 = FaultResetCmd[2];
	Motor_controlword.filed.data3 = FaultResetCmd[3];
	for (int i = 0; i < mSendMsg.len; i++) {
		mSendMsg.data[i] = Motor_controlword.sdoData[i];
	}
	cansendapp->canSend(&mSendMsg);
	SetVelocityMode(opIndex);
	return 0;
}

int MoonsCMD::SetVelocityMode(int opIndex) {

	Message mSendMsg;
	int temp, tempCodID;

	//Ready to Switch on
	if ((opIndex > 0) && (opIndex < 9)) {
		tempCodID = 0x600 + opIndex;
	} else {
		return -1;
	}
	usleep(10000);
	mSendMsg.cob_id = tempCodID;
	mSendMsg.len = 8;
	mSendMsg.rtr = 0;
	Motor_controlword.filed.data0 = ReadyToSwitchOn[0];
	Motor_controlword.filed.data1 = ReadyToSwitchOn[1];
	Motor_controlword.filed.data2 = ReadyToSwitchOn[2];
	Motor_controlword.filed.data3 = ReadyToSwitchOn[3];
	for (int i = 0; i < mSendMsg.len; i++) {
		mSendMsg.data[i] = Motor_controlword.sdoData[i];
	}
	cansendapp->canSend(&mSendMsg);

	usleep(10000);

	//Switch On
	mSendMsg.cob_id = tempCodID;
	mSendMsg.len = 8;
	mSendMsg.rtr = 0;
	Motor_controlword.filed.data0 = SwitchOn[0];
	Motor_controlword.filed.data1 = SwitchOn[1];
	Motor_controlword.filed.data2 = SwitchOn[2];
	Motor_controlword.filed.data3 = SwitchOn[3];
	for (int i = 0; i < mSendMsg.len; i++) {
		mSendMsg.data[i] = Motor_controlword.sdoData[i];
	}
	cansendapp->canSend(&mSendMsg);
	usleep(10000);

	//Operation Enabled; Motion Halted
	mSendMsg.cob_id = tempCodID;
	mSendMsg.len = 8;
	mSendMsg.rtr = 0;
	Motor_controlword.filed.data0 = OpEnableMotionHalted[0];
	Motor_controlword.filed.data1 = OpEnableMotionHalted[1];
	Motor_controlword.filed.data2 = OpEnableMotionHalted[2];
	Motor_controlword.filed.data3 = OpEnableMotionHalted[3];
	for (int i = 0; i < mSendMsg.len; i++) {
		mSendMsg.data[i] = Motor_controlword.sdoData[i];
	}
	cansendapp->canSend(&mSendMsg);
	usleep(10000);

	//‘Set to Profile Velocity Mode
	mSendMsg.cob_id = tempCodID;
	mSendMsg.len = 8;
	mSendMsg.rtr = 0;
	Motor_operationMode.filed.data0 = SetToProfileVelocity[0];
	Motor_operationMode.filed.data1 = SetToProfileVelocity[1];
	Motor_operationMode.filed.data2 = SetToProfileVelocity[2];
	Motor_operationMode.filed.data3 = SetToProfileVelocity[3];
	for (int i = 0; i < mSendMsg.len; i++) {
		mSendMsg.data[i] = Motor_operationMode.sdoData[i];
	}
	cansendapp->canSend(&mSendMsg);
	usleep(10000);
	//Set Target Velocity to 10 rps
	mSendMsg.cob_id = tempCodID;
	mSendMsg.len = 8;
	mSendMsg.rtr = 0;
	Motor_velocitySet.filed.data0 = 0x00;
	Motor_velocitySet.filed.data1 = 0x00;
	Motor_velocitySet.filed.data2 = 0;
	Motor_velocitySet.filed.data3 = 0;
	for (int i = 0; i < mSendMsg.len; i++) {
		mSendMsg.data[i] = Motor_velocitySet.sdoData[i];
	}
	cansendapp->canSend(&mSendMsg);
	usleep(10000);

	//Set Acceleration to 25 rps/s
	mSendMsg.cob_id = tempCodID;
	mSendMsg.len = 8;
	mSendMsg.rtr = 0;
	Motor_AccelerationSet.filed.data0 = 0x96;
	Motor_AccelerationSet.filed.data1 = 0x00;
	Motor_AccelerationSet.filed.data2 = 0;
	Motor_AccelerationSet.filed.data3 = 0;
	for (int i = 0; i < mSendMsg.len; i++) {
		mSendMsg.data[i] = Motor_AccelerationSet.sdoData[i];
	}
	cansendapp->canSend(&mSendMsg);
	usleep(10000);

	//Set Deceleration to 25 rps/s
	mSendMsg.cob_id = tempCodID;
	mSendMsg.len = 8;
	mSendMsg.rtr = 0;
	Motor_DecelerationSet.filed.data0 = 0x96;
	Motor_DecelerationSet.filed.data1 = 0x00;
	Motor_DecelerationSet.filed.data2 = 0;
	Motor_DecelerationSet.filed.data3 = 0;
	for (int i = 0; i < mSendMsg.len; i++) {
		mSendMsg.data[i] = Motor_DecelerationSet.sdoData[i];
	}
	cansendapp->canSend(&mSendMsg);
	usleep(10000);

	//Set start
	mSendMsg.cob_id = tempCodID;
	mSendMsg.len = 8;
	mSendMsg.rtr = 0;
	Motor_controlword.filed.data0 = StartMotion[0];
	Motor_controlword.filed.data1 = StartMotion[1];
	Motor_controlword.filed.data2 = StartMotion[2];
	Motor_controlword.filed.data3 = StartMotion[3];
	for (int i = 0; i < mSendMsg.len; i++) {
		mSendMsg.data[i] = Motor_controlword.sdoData[i];
	}
	cansendapp->canSend(&mSendMsg);
	return 0;
}

int MoonsCMD::SetStart(int opIndex) {
	Message mSendMsg;
	//Set start
	usleep(1000);
	if ((opIndex > 0) && (opIndex < 9)) {
		mSendMsg.cob_id = 0x600 + opIndex;
		mSendMsg.len = 8;
		mSendMsg.rtr = 0;
		Motor_controlword.filed.data0 = StartMotion[0];
		Motor_controlword.filed.data1 = StartMotion[1];
		Motor_controlword.filed.data2 = StartMotion[2];
		Motor_controlword.filed.data3 = StartMotion[3];
		for (int i = 0; i < mSendMsg.len; i++) {
			mSendMsg.data[i] = Motor_controlword.sdoData[i];
		}
		cansendapp->canSend(&mSendMsg);
		return 0;
	} else {
		return -1;
	}
}

int MoonsCMD::SetStop(int opIndex) {
	Message mSendMsg;
	//Set start
	if ((opIndex > 0) && (opIndex < 9)) {
		mSendMsg.cob_id = 0x600 + opIndex;
		mSendMsg.len = 8;
		mSendMsg.rtr = 0;
		Motor_controlword.filed.data0 = OpEnableMotionHalted[0];
		Motor_controlword.filed.data1 = OpEnableMotionHalted[1];
		Motor_controlword.filed.data2 = OpEnableMotionHalted[2];
		Motor_controlword.filed.data3 = OpEnableMotionHalted[3];
		for (int i = 0; i < mSendMsg.len; i++) {
			mSendMsg.data[i] = Motor_controlword.sdoData[i];
		}
		cansendapp->canSend(&mSendMsg);
		return 0;
	} else {
		return -1;
	}
}

int MoonsCMD::SetVelocity(float Spd, int opIndex) {
	Message mSendMsg;
	uint32_t uiSpd;
	if (Spd > fspdMax) {
		Spd = fspdMax;
	} else if (Spd < fspdMin) {
		Spd = fspdMin;
	}
#ifdef DGB_PRINTF_MOONSCMD
	printf("MoonsCMD Spd is: %f opIndex is %d \n",Spd,opIndex);
	printf("MoonsCMD fspdMin is: %f fspMax is %f \n",fspdMin,fspdMax);
#endif
	uiSpd = (Spd * 240) / 60;
	//Set Target Velocity to 10 rps
	if ((opIndex > 0) && (opIndex < 9)) {
		mSendMsg.cob_id = 0x600 + opIndex;
		mSendMsg.len = 8;
		mSendMsg.rtr = 0;
		Motor_velocitySet.filed.data0 = uiSpd & 0x00FF;
		Motor_velocitySet.filed.data1 = (uiSpd & 0xFF00) >> 8;
		Motor_velocitySet.filed.data2 = 0;
		Motor_velocitySet.filed.data3 = 0;
		for (int i = 0; i < mSendMsg.len; i++) {
			mSendMsg.data[i] = Motor_velocitySet.sdoData[i];
		}
		cansendapp->canSend(&mSendMsg);
		return 0;
	} else {
		return -1;
	}
}

int MoonsCMD::GetVelocity(int opIndex) {
	Message mSendMsg;
	//get Current Velocity
	if ((opIndex > 0) && (opIndex < 9)) {
		mSendMsg.cob_id = 0x600 + opIndex;
		mSendMsg.len = 8;
		mSendMsg.rtr = 0;
		Motor_CurVelocityRead.filed.data0 = 0;
		Motor_CurVelocityRead.filed.data1 = 0;
		Motor_CurVelocityRead.filed.data2 = 0;
		Motor_CurVelocityRead.filed.data3 = 0;
		for (int i = 0; i < mSendMsg.len; i++) {
			mSendMsg.data[i] = Motor_CurVelocityRead.sdoData[i];
		}
		cansendapp->canSend(&mSendMsg);
		return 0;
	} else {
		return -1;
	}
}

int MoonsCMD::GetCount(int opIndex) {
	Message mSendMsg;
	//get Current count
	if ((opIndex > 0) && (opIndex < 9)) {
		mSendMsg.cob_id = 0x600 + opIndex;
		mSendMsg.len = 8;
		mSendMsg.rtr = 0;
		Motor_CountRead.filed.data0 = 0;
		Motor_CountRead.filed.data1 = 0;
		Motor_CountRead.filed.data2 = 0;
		Motor_CountRead.filed.data3 = 0;
		for (int i = 0; i < mSendMsg.len; i++) {
			mSendMsg.data[i] = Motor_CountRead.sdoData[i];
		}
		cansendapp->canSend(&mSendMsg);
		return 0;
	} else {
		return -1;
	}
}
int MoonsCMD::GetDspAlarm(int opIndex) {
	Message mSendMsg;
	//get Current count
	if ((opIndex > 0) && (opIndex < 9)) {
		mSendMsg.cob_id = 0x600 + opIndex;
		mSendMsg.len = 8;
		mSendMsg.rtr = 0;
		Motor_DspStatusCode.filed.data0 = 0;
		Motor_DspStatusCode.filed.data1 = 0;
		Motor_DspStatusCode.filed.data2 = 0;
		Motor_DspStatusCode.filed.data3 = 0;
		for (int i = 0; i < mSendMsg.len; i++) {
			mSendMsg.data[i] = Motor_DspStatusCode.sdoData[i];
		}
		cansendapp->canSend(&mSendMsg);
		return 0;
	} else {
		return -1;
	}
}

int MoonsCMD::GetDspStatus(int opIndex) {
	Message mSendMsg;
	//get Current count
	if ((opIndex > 0) && (opIndex < 9)) {
		mSendMsg.cob_id = 0x600 + opIndex;
		mSendMsg.len = 8;
		mSendMsg.rtr = 0;
		Motor_DspStatusCode.filed.data0 = 0;
		Motor_DspStatusCode.filed.data1 = 0;
		Motor_DspStatusCode.filed.data2 = 0;
		Motor_DspStatusCode.filed.data3 = 0;
		for (int i = 0; i < mSendMsg.len; i++) {
			mSendMsg.data[i] = Motor_DspStatusCode.sdoData[i];
		}
		cansendapp->canSend(&mSendMsg);
		return 0;
	} else {
		return -1;
	}
}

int MoonsCMD::GetStatusword(int opIndex) {
	Message mSendMsg;
	//get Current count
	if ((opIndex > 0) && (opIndex < 9)) {
		mSendMsg.cob_id = 0x600 + opIndex;
		mSendMsg.len = 8;
		mSendMsg.rtr = 0;
		Motor_Statusword.filed.data0 = 0;
		Motor_Statusword.filed.data1 = 0;
		Motor_Statusword.filed.data2 = 0;
		Motor_Statusword.filed.data3 = 0;
		for (int i = 0; i < mSendMsg.len; i++) {
			mSendMsg.data[i] = Motor_Statusword.sdoData[i];
		}
		cansendapp->canSend(&mSendMsg);
		return 0;
	} else {
		return -1;
	}
}

int MoonsCMD::GetCurModes(int opIndex) {
	Message mSendMsg;
	//get Current count
	if ((opIndex > 0) && (opIndex < 9)) {
		mSendMsg.cob_id = 0x600 + opIndex;
		mSendMsg.len = 8;
		mSendMsg.rtr = 0;
		Motor_Statusword.filed.data0 = 0;
		Motor_Statusword.filed.data1 = 0;
		Motor_Statusword.filed.data2 = 0;
		Motor_Statusword.filed.data3 = 0;
		for (int i = 0; i < mSendMsg.len; i++) {
			mSendMsg.data[i] = Motor_Statusword.sdoData[i];
		}
		cansendapp->canSend(&mSendMsg);
		return 0;
	} else {
		return -1;
	}
}

int MoonsCMD::unpackReadPara(SDOIndex sdoIndex, UINT32& rdata, int opIndex) {
	int recerr = 0;
	UNS32 data;
	HEX_DATA32 hexData32;
	HEX_DATA16 hexData16;
	VCI_CAN_OBJ rec;
	for (int reReceive = 0; reReceive < 3; reReceive++) {
		usleep(2000); //delay 2ms
		recerr = cansendapp->canReceiveUnpack(rec, sdoIndex, opIndex);
		if (recerr == 0) {
#ifdef DGB_PRINTF_MOONSCMD
			for(int i = 0; i < 8;i++)
			{
				printf("MoonsCDM received response  %08x \n",rec.Data[i]);
			}
#endif
			switch (rec.Data[0]) {
			case 0x4F: //Response one Byte
				data = rec.Data[4];
				rdata = (UINT32 ) data;
#ifdef DGB_PRINTF_MOONSCMD
				printf("MoonsCDM received data value  %d \n",data);
#endif
				return 0;
			case 0x4B: //Response two Byte
				UNS16 data16;
				hexData16.field.data0 = rec.Data[4];
				hexData16.field.data1 = rec.Data[5];
				data16 = (rec.Data[4] + rec.Data[5] * 256);
//				printf("MoonsCDM rec.Data[4] %d, rec.Data[5] %d \n",rec.Data[4],rec.Data[5]);
//				#ifdef DGB_PRINTF_MOONSCMD
//				printf("MoonsCDM before trans 16data: %d \n",data16);
//				#endif
				rdata = (UINT32) hexData16.u16Data;
#ifdef DGB_PRINTF_MOONSCMD
				printf("MoonsCDM after trans 32data: %d \n",rdata);
				printf("MoonsCDM received data value  %d \n",data);
				printf("MoonsCDM (UINT32 &)data16 value  %d \n",(UINT32 &)data16);
				printf("MoonsCDM (UINT32 )data16  value  %d \n",(UINT32 )data16);
#endif
				return 0;
			case 0x47: //Response three Byte
				data = (rec.Data[4] + rec.Data[5] * 256 + rec.Data[6] * 65535);
				rdata = (UINT32) data;
#ifdef DGB_PRINTF_MOONSCMD
				printf("MoonsCDM received data value  %d \n",data);
				printf("MoonsCDM received data value  %d \n",data);
				printf("MoonsCDM received data value  %d \n",data);
#endif
				return 0;
			case 0x43: //Response four Byte
				hexData32.field.data0 = rec.Data[4];
				hexData32.field.data1 = rec.Data[5];
				hexData32.field.data2 = rec.Data[6];
				hexData32.field.data3 = rec.Data[7];
				rdata = (UNS32) hexData32.u32Data;
				#ifdef DGB_PRINTF_MOONSCMD
				ROS_WARN("Data[4]:%Xd, Data[5]:%Xd, Data[6]:%Xd, Data[7]:%Xd", rec.Data[4], rec.Data[5], rec.Data[6], rec.Data[7]);
				ROS_INFO("typedata %d",hexData32.u32Data);
				    printf("MoonsCDM Data[4]:%Xd, Data[5]:%Xd, Data[6]:%Xd, Data[7]:%Xd\n",
				    		rec.Data[4], rec.Data[5], rec.Data[6], rec.Data[7]);
					printf("MoonsCDM received data value  %d ,typedata %d\n",data,hexData32.u32Data);
				#endif
				return 0;
			case 0x80: //Response Err
				ROS_ERROR("0x80 rec.Data[0] = %d", rec.Data[0]);
				return -1;
			default:
				ROS_ERROR("default rec.Data[0] %d", rec.Data[0]);
				return -1;
			}
		}
//		else
//		{
//			#ifdef DGB_PRINTF_MOONSCMD
//		    printf("can't received response data,repeat\n");
//			#endif
//			ROS_WARN("============== can't received response data,repeat ============");
//		}
	}
	ROS_ERROR("total");
	return -1;
}

