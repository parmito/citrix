/*
 * State.h
 *
 *  Created on: 21/09/2018
 *      Author: danilo
 */

#ifndef STATE_H_
#define STATE_H_

#include "defines.h"

const sStateMachineType *psSearchEvent (const sStateMachineType *psStateTable, unsigned char ucIncoming);
void eEventHandler (unsigned char ucDest,const sStateMachineType *psStateTable, unsigned char *piState, sMessageType *psMessage);



#endif /* STATE_H_ */
