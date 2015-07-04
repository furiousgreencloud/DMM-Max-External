//
//  DmmDriver.h
//  dmmsend
//
//  Created by Ciel Marks on 27/04/2015.
//
//

#ifndef dmmsend_DmmDriver_h
#define dmmsend_DmmDriver_h

typedef enum {In_Progress = 0, Complete_Success,  CRC_Error, Timeout_Error } ProtocolError_t;

typedef struct DmmProtocolState {
    unsigned char Read_Package_Buffer[8],Read_Num,Read_Package_Length;
    unsigned char MotorPosition32Ready_Flag, MotorTorqueCurrentReady_Flag, MainGainRead_Flag;
    long Motor_Pos32, MotorTorqueCurrent, MainGain_Read;
    long Drive_Read_Value;
    char Drive_Read_Axis_ID;
    unsigned char Drive_Read_Code;
    Boolean ProtocolError;
    void (*SerialWritePtr)(char byte, void *hook); // Caller must provide this function
    void (*ReportPositionPtr)(long pos, void * hook); // Caller must provide this function
    void *hook; // Caller Can pull anything in here and it will be returned
} DmmProtocolState_t;

void MoveMotorToAbsolutePosition32(DmmProtocolState_t *pp, char Axis, long pos);
void ResetOrgin(DmmProtocolState_t* pp, char Axis);
void SetMaxAccel(DmmProtocolState_t* pp, char Axis, int maxAccel);
void SetMaxSpeed(DmmProtocolState_t* pp, char Axis, int maxSpeed);
void ReadMotorPosition32(DmmProtocolState_t *pp, char Axis);
void MoveMotorConstantRotation(DmmProtocolState_t* pp, char Axis_Num,long r);
void ReadPackage(DmmProtocolState_t* pp, unsigned char c);
#endif
