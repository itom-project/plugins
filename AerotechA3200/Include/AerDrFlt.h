/*
   This header file is used by 960 and Axis object
   and defines the strings for the fault messages
   This file is NOT to be used by user applications.
*/
#ifndef __AER_DR_FLT_H__
#define __AER_DR_FLT_H__

#include "aertdef.h"

TCHAR FaultMsg[32][AER_PROG_MSG_SIZE] =
{
               "Position error",       //    BDRV_FAULT_POSERR:
               "Over current",         //    BDRV_FAULT_CURERR:
               "CW EOT limit",         //    BDRV_FAULT_CWHARDLIM:
               "CCW EOT limit",        //    BDRV_FAULT_CCWHARDLIM:
               "CW soft limit",        //    BDRV_FAULT_CWSOFTLIM:
               "CCW soft limit",       //    BDRV_FAULT_CCWSOFTLIM:
               "Amplifier fault",      //    BDRV_FAULT_AMPPOWER:
               "Position fbk",         //    BDRV_FAULT_POSFBK:
               "Velocity fbk",         //    BDRV_FAULT_VELFBK:
               "Hall fault",           //    BDRV_FAULT_HALLFBK:
               "Max velocity cmd",     //    BDRV_FAULT_MAXVEL:
               "ESTOP fault",          //    BDRV_FAULT_ESTOP:
               "Velocity error",       //    BDRV_FAULT_MAXVELERR:
               "Task fault",           //    BDRV_FAULT_TASK:
               "Probe fault",          //    BDRV_FAULT_PROBE:
               "Auxiliary fault",      //    BDRV_FAULT_AUXILIARY:
               "Safe zone fault",      //    BDRV_FAULT_SAFEZONE:
               "Motor temp",           //    BDRV_FAULT_MOTOR_TEMP:
               "Amplifier temp",       //    BDRV_FAULT_AMP_TEMP:
               "Ext encoder fault",    //    BDRV_FAULT_EXTERNAL:
               "Comm lost fault",      //    BDRV_FAULT_COMMUN:
               "SPARE21",
               "SPARE22",
               "SPARE23",
               "SPARE24",
               "SPARE25",
               "SPARE26",
               "SPARE27",
               "SPARE28",
               "SPARE29",
               "SPARE30",
               "SPARE31",
};
#endif
