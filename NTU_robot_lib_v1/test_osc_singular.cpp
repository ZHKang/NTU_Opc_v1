#include <iostream>
#include <Windows.h>
#include "Robot_main.h"
#include <fstream>
#include <time.h>
// ------------  robot library ----------
#include "roboop\robot.h"
#include "roboop\controller.h"
#include "roboop\control_select.h"
#include "roboop\trajectory.h"
#include "roboop\utils.h"
#include "roboop\quaternion.h"
#include "roboop\dynamics_sim.h"
//  for V-REP header
#include <stdio.h>
#include <stdlib.h>

extern "C" {
#include "extApi.h"
}
using namespace std;
#ifdef use_namespace
using namespace ROBOOP;
#endif
//  Global Varaibles
Robot robot_atom;
int My_robot_TorqueControl::NullspaceControl()
{

	return 0;
}
int My_robot_TorqueControl::InitialAtom()
{
	const Real Atom_data_DH[] =
	{
		// joint_type, theta, d,a,alpha, thetamin, thetamax, joint_offset,/**/ m, cm x, cm y, cm z,/**/ Ixx, Ixy, Ixz, Iyy,Iyz, Izz, lock
		0, 0, 1.22, 0, M_PI / 2, 0, 0, 0,		  /* */ 1.529, 0, 0, -0.01,		/* */ 0.16, 0, 0, 0.16, 0, 0.16, 0,
		0, 0, 0, 0, M_PI / 2, 0, 0, -M_PI,	  /* */ 0.344, 0, 0, 0.02,    /* */0.16, 0, 0, 0.16, 0, 0.16, 0,
		0, 0, 0.371, 0.01, M_PI / 2, 0, 0, M_PI,      /* */ 2.033, 0, -0.13, 0,    /* */ 0.16, 0, 0, 0.16, 0, 0.16, 0,
		0, 0, 0, -0.01, -M_PI / 2, 0, 0, -M_PI / 2,    /* */ 0.236, 0, 0, 0.03,    /* */ 0.16, 0, 0, 0.16, 0, 0.16, 0,
		0, 0, 0.28, 0, M_PI / 2, 0, 0, M_PI,     /* */ 1.331, 0, -0.13, 0,    /* */ 0.08, 0, 0, 0.08, 0, 0.08, 0,
		0, 0, 0, 0.1451, M_PI / 2, 0, 0, M_PI / 2,    /* */ 0.274, 0, 0.03, 0,   /* */ 0.08, 0, 0, 0.08, 0, 0.08, 0
	};
	const Real Atom_motor[] =  // Im Gr B Cf [3863 3257 2642 2642 2642 2232]
	{
		1.2e-5, 200, 0, 0, // using + and - directions average
		1.2e-5, 250, 0, 0,
		1.1e-6, 200, 0, 0,
		1.1e-6, 250, 0, 0,
		1.1e-6, 200, 0, 0,
		3.8e-7, 300, 0, 0
	};

	Matrix initrobot, initrobotm;
	initrobot = Matrix(6, 19);
	initrobotm = Matrix(6, 4);
	initrobot << Atom_data_DH;
	initrobotm << Atom_motor;
	robot_atom = Robot(initrobot, initrobotm);   //  Get a Robot Model
	return 1;
}
int main(int argc, char* argv[])
{
	int portNb = 0;
	int atom_jointHandle[6];
	int atom_targetHandle = 0;
	int atom_baseHandle = 0;
	double Startime, Stoptime;
	if (argc >= 10)
	{
		portNb = atoi(argv[1]);
		atom_jointHandle[0] = atoi(argv[2]);
		atom_jointHandle[1] = atoi(argv[3]);
		atom_jointHandle[2] = atoi(argv[4]);
		atom_jointHandle[3] = atoi(argv[5]);
		atom_jointHandle[4] = atoi(argv[6]);
		atom_jointHandle[5] = atoi(argv[7]);
		atom_targetHandle = atoi(argv[8]);
		atom_baseHandle = atoi(argv[9]);
		printf("Port Number = %d \n", portNb);
	}
	else
	{
		printf("Indicate following arguments: 'portNumber!\n");
		extApi_sleepMs(5000);
		return 0;
	}
	// Initial Robot
	My_robot_TorqueControl CMyRobot;
	if (CMyRobot.InitialAtom())
		printf("Atom Model done %d DOF \n", robot_atom.get_available_dof());

	int clientID = simxStart((simxChar*)"127.0.0.1", portNb, true, true, 2000, 5);
	if (clientID != -1)
	{
		Real atom_theta_new = 0;
		float *atom_theta = new float[6];
		float *target_pos = new float[3];
		float *target_orient = new float[3];
		float *atom_set_qn = new float[6];
		float *atom_get_qn = new float[1];
		float *atom_get_qdn = new float[1];
		float *atom_get_torque = new float[1];
		Matrix Tn(4, 4);
		ColumnVector qn(6), qdn(6), qddn(6), torque(6);
		//ofstream TorqueSave("TorqueVREPSave.txt");
		//ofstream TorqueSave1("TorqueNTUSave.txt");
		// initial robot move to an unsigular position
		for (int i = 0; i < 6; i++){
			simxGetJointPosition(clientID, atom_jointHandle[i], atom_get_qn, simx_opmode_streaming);
			qn(i + 1) = *atom_get_qn;
		}
		robot_atom.set_q(qn);

		Tn = robot_atom.kine();
		while (simxGetConnectionId(clientID) != -1)
		{
			Startime = clock();
			// Get Trajectory Target based on atom base dummy
			simxGetObjectPosition(clientID, atom_targetHandle, atom_baseHandle, target_pos, simx_opmode_streaming);
			simxGetObjectOrientation(clientID, atom_targetHandle, atom_baseHandle, target_orient, simx_opmode_streaming);
			simxPauseCommunication(clientID, 1);
			for (int i = 0; i < 6; i++){
				simxSetJointTargetPosition(clientID, atom_jointHandle[i], (float)atom_set_qn[i], simx_opmode_oneshot);
				simxGetJointForce(clientID, atom_jointHandle[i], atom_get_torque, simx_opmode_streaming);
				simxGetJointPosition(clientID, atom_jointHandle[i], atom_get_qn, simx_opmode_streaming);
				qn(i + 1) = *atom_get_qn;
				//TorqueSave << *atom_get_torque << "\t";
			}
			torque = robot_atom.torque(qn, qdn, qddn);
			simxPauseCommunication(clientID, 0);
			extApi_sleepMs(5);
			Stoptime = clock();
			//cout << (Stoptime - Startime)/CLOCKS_PER_SEC << endl;
		}
		simxFinish(clientID);
	}
	return 0;
}