/**
 * @file ma_jobParser.h
 * @brief Contains all details of the job parser engine
 * @author Jack Cripps // jackc@monitoraudio.com
 * @date 21/10/2019
 */

#include "ma_jobParser.h"

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>

#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))


enum state_code lookupTransition(enum state_code state, enum ret_code ret)
{
	enum state_code nextState = fsmError;

	for (uint8_t i = 0; i < COUNT_OF(stateTransitions); ++i)
	{
		if ((stateTransitions[i].srcState == state) && (stateTransitions[i].retCode == ret))
		{
			nextState = stateTransitions[i].destState;
			break;
		}
	}

	return nextState;
}


int entryState(void)
{
	enum ret_code ret = failure;

	return ret;
}


int queryState(void)
{
	enum ret_code ret = failure;

	return ret;
}


int describeState(void)
{
	enum ret_code ret = failure;

	return ret;
}


int executeState(void)
{
	enum ret_code ret = failure;

	return ret;
}


int finishState(void)
{
	enum ret_code ret = failure;

	return ret;
}

