/*
 * RapiData.h
 *
 *  Created on: 27 Nov 2018
 *      Author: Lauri
 */

#ifndef RAPIDATA_H_
#define RAPIDATA_H_

#include <string>
class RapiData {
public:
	RapiData(int tDir=0, int rDir=0, int nLDR=0, int sLDR=0, int wLDR=0, int eLDR=0) :
	tiltDir(tDir), rotateDir(rDir), northLDR(nLDR), southLDR(sLDR), westLDR(wLDR), eastLDR(eLDR)
	{}

	virtual ~RapiData();
	std::string getStringRapiData();



	int tiltDir;	//1 is up, 0 is down
	int rotateDir; //1 is clockwise, 0 is counterclockwise
	int northLDR; // data values from adc
	int southLDR;
	int westLDR;
	int eastLDR;
};

#endif /* RAPIDATA_H_ */
