//
//  DmmDriver.h
//  dmmsend
//
//  Created by Ciel Marks on 27/04/2015.
//
//

#ifndef dmmsend_DmmDriver_h
#define dmmsend_DmmDriver_h

void SerialWrite(char c, void* hook);

typedef enum {In_Progress = 0, Complete_Success,  CRC_Error, Timeout_Error } ProtocolError_t;

typedef struct DmmProtocolBuffer {
    char InputBuffer[256]; //Input buffer from RS232,
    char OutputBuffer[256]; //Output buffer to RS232,
    signed int InBfTopPointer,InBfBtmPointer;//input buffer pointers
    unsigned int OutBfTopPointer,OutBfBtmPointer;//output buffer pointers
    unsigned char Read_Package_Buffer[8],Read_Num,Read_Package_Length;
    unsigned char MotorPosition32Ready_Flag, MotorTorqueCurrentReady_Flag, MainGainRead_Flag;
    long Motor_Pos32, MotorTorqueCurrent, MainGain_Read;
    long Drive_Read_Value;
    unsigned char Drive_Read_Code;
    Boolean ProtocolError;
    void (*SerialWriteFunctionPtr)(char, void *); // Caller must provide this function
    void *hook; // Caller Can pull anything in here and it will be returned

} DmmProtocolBuffer_t;

void MoveMotorToAbsolutePosition32(DmmProtocolBuffer_t *pp, char Axis_Num, long pos);
void ResetOrgin(DmmProtocolBuffer_t* pp, char Axis_Num);

#endif
