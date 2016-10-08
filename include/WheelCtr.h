#ifndef WHEELCTR_H
#define WHEELCTR_H

#include "moons/applicfg.h"
#include <ros/ros.h>
#include <ros/time.h>
#include "moons/MoonsCmd.h"



class WheelCtr
{
public:
	WheelCtr();
    ~WheelCtr();

	/// Set Velocity
	void setVelocities(float s_w1, float s_w2);

	/// Get Velocity
	void getVelocities(float& g_w1, float& g_w2);

	/// Get Position
	void getPositions(INTEGER32& p_w1, INTEGER32& p_w2);

	/// Set Start
	void setStart();

	/// Ack Motor modes
	int ackMotormodes(int opIndex);

    long period1;
    pthread_t moonsCmdEmg_thread;

    MoonsCMD *moonscmdapp;

};
#endif // CANSEND_H
