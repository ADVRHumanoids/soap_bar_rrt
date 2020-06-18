// Copyright 2006-2013 Dr. Marc Andreas Freese. All rights reserved.
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
//
// -------------------------------------------------------------------
// This file is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
// You are free to use/modify/distribute this file for whatever purpose!
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.0.3 on April 29th 2013

#include "v_repExtMultiContactPlanner.h"
#include "v_repLib.h"
#include "constant_values.hpp"
#include <map>
#include <ctime>
#include <fenv.h>
#include <time.h>
#include <fstream>
#include <stdio.h>
#include <algorithm>
#include <vector>
#include <boost/lexical_cast.hpp>
#include <unistd.h>
#include <iostream>
#include "enum.h"
#include "termcolor.hpp"
#include <random>
#include <string>
#include "Planner.hpp"


Planner* planner;

simInt hL_FOOT, hR_FOOT, hL_HAND, hR_HAND, hCOM, hGOAL;
int nJoints = 20;

std::vector<Eigen::Vector3d> rCoMPlan;
int iPlan;
int planSize;
int iFrame; 
std::vector<Eigen::Vector3d> rLHPlan;
std::vector<Eigen::Vector3d> rRHPlan;
std::vector<Eigen::Vector3d> rLFPlan;
std::vector<Eigen::Vector3d> rRFPlan;

Eigen::Vector3d getNormalAtPoint(Eigen::MatrixXd pointCloud, Eigen::Vector3d p){

	// fitting a plane on k nearest points to p 
	std::map<double, Eigen::Vector3d> pointsOrderedByDistance;
	for(int i = 0; i < pointCloud.rows(); i++){
		Eigen::Vector3d p_i = pointCloud.row(i).transpose();
		double d = euclideanDistance(p_i, p);
		pointsOrderedByDistance.insert(std::pair<double, Eigen::Vector3d>(d, p_i)); 
	}

	int k = 100;
	Eigen::MatrixXd pointsNear(k,3);
	std::map<double, Eigen::Vector3d>::iterator itr; 
    itr = pointsOrderedByDistance.begin();
	for(int i = 0; i < k; i++) { 
		double d = itr->first;
		Eigen::Vector3d p_i = itr->second;
		pointsNear.row(i) = p_i.transpose();	
		itr++;
    } 
	Eigen::Vector3d centroid(pointsNear.col(0).mean(), pointsNear.col(1).mean(), pointsNear.col(2).mean());
	pointsNear.col(0).array() -= centroid(0);
	pointsNear.col(1).array() -= centroid(1);
	pointsNear.col(2).array() -= centroid(2);
	auto svd = pointsNear.transpose().jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
	
	// computing the normal in p	
	Eigen::Vector3d normal = svd.matrixU().rightCols<1>();
	if((Eigen::Vector3d(0.0, 0.0, 2.0)-p).transpose()*normal < 0.0) normal = -normal;	
	
	return normal;
}

void initializeSimulation(){

	std::cout << "CHOOSE OPTION:" << std::endl;
	std::cout << "	1) plan" << std::endl;
	std::cout << "	2) show" << std::endl;
	
	int option;
	std::cin >> option;
	if (option == 1) {

		std::cout << "Initializing Simulation" << std::endl;

		simFloat posL_FOOT[3];
		simFloat posR_FOOT[3];
		simFloat posL_HAND[3];
		simFloat posR_HAND[3];
		simFloat posCOM[3];
		simFloat posGOAL[3];
		hL_FOOT = simGetObjectHandle("L_FOOT");
		hR_FOOT = simGetObjectHandle("R_FOOT");
		hL_HAND = simGetObjectHandle("L_HAND");
		hR_HAND = simGetObjectHandle("R_HAND");
		hCOM = simGetObjectHandle("COM");
		hGOAL = simGetObjectHandle("GOAL");
		simGetObjectPosition(hL_FOOT, -1, posL_FOOT);
		simGetObjectPosition(hR_FOOT, -1, posR_FOOT);
		simGetObjectPosition(hL_HAND, -1, posL_HAND);
		simGetObjectPosition(hR_HAND, -1, posR_HAND);
		simGetObjectPosition(hCOM, -1, posCOM);
		simGetObjectPosition(hGOAL, -1, posGOAL);

		std::cout << "posL_FOOT = " << posL_FOOT[0] << " " <<  posL_FOOT[1] << " " <<  posL_FOOT[2] << std::endl;
		std::cout << "posR_FOOT = " << posR_FOOT[0] << " " <<  posR_FOOT[1] << " " <<  posR_FOOT[2] << std::endl;
		std::cout << "posL_HAND = " << posL_HAND[0] << " " <<  posL_HAND[1] << " " <<  posL_HAND[2] << std::endl;
		std::cout << "posR_HAND = " << posR_HAND[0] << " " <<  posR_HAND[1] << " " <<  posR_HAND[2] << std::endl;
		std::cout << "posCOM = " << posCOM[0] << " " <<  posCOM[1] << " " <<  posCOM[2] << std::endl;
		std::cout << "posGOAL = " << posGOAL[0] << " " <<  posGOAL[1] << " " <<  posGOAL[2] << std::endl;

		// retrieve start

		Configuration qInit;
		Eigen::Vector3d pFB(posCOM[0], posCOM[1], posCOM[2]);
		Eigen::Vector3d eFB(0.0, 0.0, 0.0);
		Eigen::VectorXd jnt(nJoints);
		jnt.setZero();
		qInit.setFBPosition(pFB);
		qInit.setFBOrientation(eFB);
		qInit.setJointValues(jnt);	

		std::vector<EndEffector> activeEEsInit;
		activeEEsInit.push_back(L_FOOT);
		activeEEsInit.push_back(R_FOOT); 

		Eigen::MatrixXd posActiveEEsInit(activeEEsInit.size(), 3);
		posActiveEEsInit.row(0) = Eigen::Vector3d(posL_FOOT[0],  posL_FOOT[1],  posL_FOOT[2]).transpose();
		posActiveEEsInit.row(1) = Eigen::Vector3d(posR_FOOT[0],  posR_FOOT[1],  posR_FOOT[2]).transpose();

		Eigen::MatrixXd contactForcesInit(activeEEsInit.size(), 3);
		contactForcesInit.row(0) = Eigen::Vector3d(0.0, 2.35, 49.0).transpose();
		contactForcesInit.row(1) = Eigen::Vector3d(0.0, -2.35, 49.0).transpose();
		
		// construct goal 

		std::vector<EndEffector> activeEEsGoal;
		//activeEEsGoal.push_back(L_FOOT);
		activeEEsGoal.push_back(L_HAND);
		
		Eigen::MatrixXd posActiveEEsGoal(activeEEsGoal.size(), 3);
		posActiveEEsGoal.row(0) = Eigen::Vector3d(posGOAL[0],  posGOAL[1],  posGOAL[2]).transpose();
		
		// construct the environment description	

		simInt pointCloudHandle = simGetObjectHandle("Point_cloud");
		simInt ptCnt;
	 	const float* cloud = simGetPointCloudPoints(pointCloudHandle, &ptCnt, NULL);

		std::cout << "point cloud size = " << ptCnt << std::endl;

		Eigen::MatrixXd pointCloud(ptCnt, 3);
		for(int i = 0; i < ptCnt; i++){
			pointCloud(i, 0) = (double)cloud[3*i];
			pointCloud(i, 1) = (double)cloud[3*i+1];
			pointCloud(i, 2) = (double)cloud[3*i+2];  
		}

		Eigen::MatrixXd pointNormals(ptCnt, 3);
		/*
		for(int i = 0; i < ptCnt; i++){
			Eigen::Vector3d p = pointCloud.row(i);
			Eigen::Vector3d n = getNormalAtPoint(pointCloud, p);
			pointNormals.row(i) = n.transpose();			
		}
		*/

		// create/initialize the planner

		planner = new Planner(qInit, posActiveEEsInit, contactForcesInit, activeEEsInit, posActiveEEsGoal, activeEEsGoal, pointCloud, pointNormals);
		std::cout << "planner created!" << std::endl;

		// run the planner

		planner->run();
		std::cout << "planner terminated!" << std::endl;

		// retrieve the solution

		rCoMPlan.clear();
		rLHPlan.clear();
		rRHPlan.clear();
		rLFPlan.clear();
		rRFPlan.clear();

		iPlan = 0;
		iFrame = 0;
		planSize = 0;

		std::vector<Stance> sigmaList;
		std::vector<Configuration> qList;
		bool solFound = planner->retrieveSolution(sigmaList, qList);
		if(solFound) std::cout << "PLAN FOUND!" << std::endl;
		else std::cout << "PLAN NOT FOUND!" << std::endl;

		std::cout << "sigmaList.size() = " << sigmaList.size() << std::endl;
		std::cout << "qList.size() = " << qList.size() << std::endl;

		planSize = qList.size();

        std::vector<EndEffector> endEffectorsList;
		endEffectorsList.push_back(L_HAND);
		endEffectorsList.push_back(R_HAND);
		//endEffectorsList.push_back(L_KNEE);
		//endEffectorsList.push_back(R_KNEE);
		endEffectorsList.push_back(L_FOOT);
		endEffectorsList.push_back(R_FOOT);			
		for(int i = 0; i < planSize; i++){
			rCoMPlan.push_back(qList.at(i).getFBPosition());
	
			Stance sigma_i = sigmaList.at(i);
			std::vector<EndEffector> activeEEs_i = sigma_i.retrieveActiveEndEffectors();

			if(!sigma_i.isActiveEndEffector(L_HAND)) rLHPlan.push_back(Eigen::Vector3d(0.0,0.0,-1000.0)); 
			if(!sigma_i.isActiveEndEffector(R_HAND)) rRHPlan.push_back(Eigen::Vector3d(0.0,0.0,-1000.0)); 
			if(!sigma_i.isActiveEndEffector(L_FOOT)) rLFPlan.push_back(Eigen::Vector3d(0.0,0.0,-1000.0)); 
			if(!sigma_i.isActiveEndEffector(R_FOOT)) rRFPlan.push_back(Eigen::Vector3d(0.0,0.0,-1000.0)); 

			for(int k = 0; k < activeEEs_i.size(); k++){
				if(activeEEs_i.at(k) == L_HAND) rLHPlan.push_back(sigma_i.getContact(k)->getPosition());
				if(activeEEs_i.at(k) == R_HAND) rRHPlan.push_back(sigma_i.getContact(k)->getPosition());
				if(activeEEs_i.at(k) == L_FOOT) rLFPlan.push_back(sigma_i.getContact(k)->getPosition());
				if(activeEEs_i.at(k) == R_FOOT) rRFPlan.push_back(sigma_i.getContact(k)->getPosition());
			}	
		}
		
	}else{
		iPlan = 0;
		iFrame = 0;
	}
}


void mainLoop(){
	//std::cout << "Running Simulation" << std::endl;
	
	if(iPlan < planSize){

		Eigen::Vector3d rCoM = rCoMPlan.at(iPlan);
		Eigen::Vector3d rLH = rLHPlan.at(iPlan);
		Eigen::Vector3d rRH = rRHPlan.at(iPlan);
		Eigen::Vector3d rLF = rLFPlan.at(iPlan);
		Eigen::Vector3d rRF = rRFPlan.at(iPlan);

		simFloat posCOM[3] = {rCoM(0), rCoM(1), rCoM(2)};
		simFloat posL_HAND[3] = {rLH(0), rLH(1), rLH(2)};
		simFloat posR_HAND[3] = {rRH(0), rRH(1), rRH(2)};
		simFloat posL_FOOT[3] = {rLF(0), rLF(1), rLF(2)};
		simFloat posR_FOOT[3] = {rRF(0), rRF(1), rRF(2)};
		
		simSetObjectPosition(hCOM, -1, posCOM);
		simSetObjectPosition(hL_HAND, -1, posL_HAND);
		simSetObjectPosition(hR_HAND, -1, posR_HAND);
		simSetObjectPosition(hL_FOOT, -1, posL_FOOT);
		simSetObjectPosition(hR_FOOT, -1, posR_FOOT);

		if(iFrame > 200){
			iFrame = 0;
			iPlan++;
		}
		iFrame++;
	}

	
}





#ifdef _WIN32
    #include <shlwapi.h>
    #pragma comment(lib, "Shlwapi.lib")
#endif /* _WIN32 */

#if defined (__linux) || defined (__APPLE__)
    #include <string.h>
    #include <sys/time.h>
    #define _stricmp(x,y) strcasecmp(x,y)
#endif

#define PLUGIN_VERSION 1
#define EIGEN_INITIALIZE_MATRICES_BY_ZERO

LIBRARY vrepLib; // the V-REP library that we will dynamically load and bind


// This is the plugin start routine (called just once, just after the plugin was loaded):

VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer, int reservedInt) {
    // Dynamically load and bind V-REP functions:
    // ******************************************
    // 1. Figure out this plugin's directory:
    char curDirAndFile[1024];
#ifdef _WIN32
    GetModuleFileName(NULL, curDirAndFile, 1023);
    PathRemoveFileSpec(curDirAndFile);
#elif defined (__linux) || defined (__APPLE__)
    getcwd(curDirAndFile, sizeof (curDirAndFile));
#endif
    std::string currentDirAndPath(curDirAndFile);
    // 2. Append the V-REP library's name:
    std::string temp(currentDirAndPath);
#ifdef _WIN32
    temp += "\\v_rep.dll";
#elif defined (__linux)
    temp += "/libv_rep.so";
#elif defined (__APPLE__)
    temp += "/libv_rep.dylib";
#endif /* __linux || __APPLE__ */
    // 3. Load the V-REP library:
    vrepLib = loadVrepLibrary(temp.c_str());
    if (vrepLib == NULL) {
        std::cout << "Error, could not find or correctly load the V-REP library. Cannot start 'PluginSkeleton' plugin.\n";
        return (0); // Means error, V-REP will unload this plugin
    }
    if (getVrepProcAddresses(vrepLib) == 0) {
        std::cout << "Error, could not find all required functions in the V-REP library. Cannot start 'PluginSkeleton' plugin.\n";
        unloadVrepLibrary(vrepLib);
        return (0); // Means error, V-REP will unload this plugin
    }
    // ******************************************

    // Check the version of V-REP:
    // ******************************************
    int vrepVer;
    simGetIntegerParameter(sim_intparam_program_version, &vrepVer);
    if (vrepVer < 20604) // if V-REP version is smaller than 2.06.04
    {
        std::cout << "Sorry, your V-REP copy is somewhat old. Cannot start 'PluginSkeleton' plugin.\n";
        unloadVrepLibrary(vrepLib);
        return (0); // Means error, V-REP will unload this plugin
    }
    // ******************************************

    simLockInterface(1);

    // Here you could handle various initializations
    // Here you could also register custom Lua functions or custom Lua constants
    // etc.

    return (PLUGIN_VERSION); // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
}

// This is the plugin end routine (called just once, when V-REP is ending, i.e. releasing this plugin):

VREP_DLLEXPORT void v_repEnd() {
    // Here you could handle various clean-up tasks

    unloadVrepLibrary(vrepLib); // release the library
}

// This is the plugin messaging routine (i.e. V-REP calls this function very often, with various messages):

VREP_DLLEXPORT void* v_repMessage(int message, int* auxiliaryData, void* customData, int* replyData) { // This is called quite often. Just watch out for messages/events you want to handle
    // Keep following 6 lines at the beginning and unchanged:
    simLockInterface(1);
    static bool refreshDlgFlag = true;
    int errorModeSaved;
    simGetIntegerParameter(sim_intparam_error_report_mode, &errorModeSaved);
    simSetIntegerParameter(sim_intparam_error_report_mode, sim_api_errormessage_ignore);
    void* retVal = NULL;

    // Here we can intercept many messages from V-REP (actually callbacks). Only the most important messages are listed here.
    // For a complete list of messages that you can intercept/react with, search for "sim_message_eventcallback"-type constants
    // in the V-REP user manual.

    if (message == sim_message_eventcallback_refreshdialogs)
        refreshDlgFlag = true; // V-REP dialogs were refreshed. Maybe a good idea to refresh this plugin's dialog too

    if (message == sim_message_eventcallback_menuitemselected) { // A custom menu bar entry was selected..
        // here you could make a plugin's main dialog visible/invisible
    }

    if (message == sim_message_eventcallback_instancepass) { // This message is sent each time the scene was rendered (well, shortly after) (very often)
        // It is important to always correctly react to events in V-REP. This message is the most convenient way to do so:

        int flags = auxiliaryData[0];
        bool sceneContentChanged = ((flags & (1 + 2 + 4 + 8 + 16 + 32 + 64 + 256)) != 0); // object erased, created, model or scene loaded, und/redo called, instance switched, or object scaled since last sim_message_eventcallback_instancepass message
        bool instanceSwitched = ((flags & 64) != 0);

        if (instanceSwitched) {
            // React to an instance switch here!!
        }

        if (sceneContentChanged) { // we actualize plugin objects for changes in the scene

            //...

            refreshDlgFlag = true; // always a good idea to trigger a refresh of this plugin's dialog here
        }
    }

    if (message == sim_message_eventcallback_mainscriptabouttobecalled) { // The main script is about to be run (only called while a simulation is running (and not paused!))
	//cout << "ON-LINE PART" << endl;	??
    }

    if (message == sim_message_eventcallback_simulationabouttostart) { // Simulation is about to start
	//cout << "OFF-LINE PART" << endl;
	

		initializeSimulation();
	

	}

    if (message == sim_message_eventcallback_simulationended) { // Simulation just ended
		std::cout << "STOP BUTTON" << std::endl;	
    }

    if (message == sim_message_eventcallback_moduleopen) { // A script called simOpenModule (by default the main script). Is only called during simulation.
        if ((customData == NULL) || (_stricmp("PluginSkeleton", (char*) customData) == 0)) // is the command also meant for this plugin?
        {
            // we arrive here only at the beginning of a simulation
        }
    }

    if (message == sim_message_eventcallback_modulehandle) { // A script called simHandleModule (by default the main script). Is only called during simulation.
	
		mainLoop();

        if ((customData == NULL) || (_stricmp("PluginSkeleton", (char*) customData) == 0)) // is the command also meant for this plugin?
        {
            // we arrive here only while a simulation is running
        }
    }

    if (message == sim_message_eventcallback_moduleclose) { // A script called simCloseModule (by default the main script). Is only called during simulation.
        if ((customData == NULL) || (_stricmp("PluginSkeleton", (char*) customData) == 0)) // is the command also meant for this plugin?
        {
            // we arrive here only at the end of a simulation
        }
    }

    if (message == sim_message_eventcallback_instanceswitch) { // Here the user switched the scene. React to this message in a similar way as you would react to a full
        // scene content change. In this plugin example, we react to an instance switch by reacting to the
        // sim_message_eventcallback_instancepass message and checking the bit 6 (64) of the auxiliaryData[0]
        // (see here above)

    }

    if (message == sim_message_eventcallback_broadcast) { // Here we have a plugin that is broadcasting data (the broadcaster will also receive this data!)

    }

    if (message == sim_message_eventcallback_scenesave) { // The scene is about to be saved. If required do some processing here (e.g. add custom scene data to be serialized with the scene)

    }

    // You can add many more messages to handle here

    if ((message == sim_message_eventcallback_guipass) && refreshDlgFlag) { // handle refresh of the plugin's dialogs
        // ...
        refreshDlgFlag = false;
    }
   
	// Play button is pressed
    if (simGetSimulationState()==17){  
  		
    }
	// Stop button is pressed
	else if(simGetSimulationState()==sim_simulation_stopped){    
		
	}  


    // Keep following unchanged:
    simSetIntegerParameter(sim_intparam_error_report_mode, errorModeSaved); // restore previous settings
    simLockInterface(0);
    return retVal;
}
