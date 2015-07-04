/*

Sample Code Notes:
(1) The sample code uses a ring buffer structure to input and output data packet bytes. Two separate ring buffers are
using in the code as char InputBuffer[256] and char OutputBuffer[256].
Two position pointers are used in each buffer structure to index the data inside the buffer structure. For example, when
a data packet is received from the servo drive, each byte received is sequentially saved into the InputBuffer with the
InBfTopPointer incremented each time. This is done until the host hardware RS232 receiver buffer is empty, meaning
all packet bytes have been read and stored. Data is processed as first-in-first-out (FIFO) queue and starts at the index
of InBfBtmPointer. InBfBtmPointer is incremented each time a byte is processed until InBfBtmPointer=InBfTopPointer,
meaning all packet bytes have been processed.

Communication Format
	Baud Rate 38400
	Start/Stop Bit	1
	Odd/Even Verify Bit	No
	Data	8-bit
	Full Duplex
	Asynchronous
	Voltage	TTL/CMOS/5V

*/

#include <sysexits.h>
#include <stdio.h>
#include <limits.h>

#include "DmmDriver.h"

#define Go_Absolute_Pos 0x01
#define Turn_ConstSpeed 0x0a
#define Set_Origin 0x00
#define Set_HighSpeed 0x14
#define Set_HighAccel 0x15
#define Set_MainGain  0x10
#define Set_SpeedGain 0x11
#define Set_IntGain  0x12


#define Set_Drive_Config 0x07
#define Config_Bit_MOTOR_DRIVE 0x10 // HIGH: Motor Drive Enabled, LOW: Motor Drive Free

#define General_Read 0x0e

#define Is_AbsPos32 0x1b
#define Is_TrqCurrent 0x1e
#define Is_MainGain 0x10
#define Is_SpeedGain 0x11
#define Is_IntGain 0x12
#define Is_Status 0x19
#define Is_Config 0x1a
#define Is_PosOn_Range 0x17
#define Is_GearNumber 0x18
#define Is_TrqCons 0x13
#define Is_HighSpeed 0x14
#define Is_HighAccel 0x15
#define Is_Drive_ID 0x16

#define Read_MainGain 0x18
#define Read_SpeedGain 0x19
#define Read_IntGain 0x20
#define Read_DriveConfig 0x08
#define Read_Drive_Status 0x09
#define Read_Pos_OnRange 0x1e
#define Read_GearNumber 0x1f
#define Read_Drive_ID 0x06

#ifndef MIN
    #define MIN(a,b) ((a<b) ? a : b)
#endif

#ifndef MAX
    #define MAX(a,b) ((a>b) ? a : b)
#endif

// Forwards
ProtocolError_t Get_Function(DmmProtocolState_t*);
long Cal_SignValue(unsigned char One_Package[8]);
unsigned int Cal_UnsignedValue(unsigned char One_Package[8]);
void Make_CRC_Send(DmmProtocolState_t *pp, unsigned char Plength,unsigned char B[8]);

const char * ParameterName(char isCode) {
    switch(isCode) {
        case Is_MainGain : return "Main Gain";
        case Is_SpeedGain : return "Speed Gain";
        case Is_IntGain : return "Intergration Gain";
        case Is_Status : return "Status Byte";
        case Is_Config : return "Config Byte";
        case Is_PosOn_Range : return "Position On Range";
        case Is_GearNumber : return "Gear Number";
        case Is_AbsPos32 : return "Absolute Position";
        case Is_TrqCons : return "Torque Contant";
        case Is_HighSpeed : return "Max Speed";
        case Is_HighAccel : return "Max Acceleration";
        case Is_Drive_ID : return "Drive ID";
        default: return "Unknown Parameter";
    }
}

void ReadPackage(DmmProtocolState_t* pp, unsigned char c) {
    unsigned char cif = c & 0x80; // Start or "End" Frame Char
    if( cif == 0) {
      pp->Read_Num = 0;
      pp->Read_Package_Length = 0;
    }
    if(cif==0 || (pp->Read_Num > 0  && ((pp->Read_Num) < sizeof(pp->Read_Package_Buffer))) ) {
      pp->Read_Package_Buffer[pp->Read_Num] = c;
      pp->Read_Num++;
      if(pp->Read_Num == 2 ) {
        cif = c>>5;
        cif = cif&0x03;
        pp->Read_Package_Length = 4 + cif;
        c = 0;
      }
      if(pp->Read_Num==pp->Read_Package_Length) {
        pp->ProtocolError = Get_Function(pp);
      }
    }
}


ProtocolError_t Get_Function(DmmProtocolState_t* pp)
{
  char ID = -1,ReceivedFunction_Code = -1, CRC_Check = -1;
  long value = -1;
  ID = pp->Read_Package_Buffer[0]&0x7f;
  ReceivedFunction_Code = pp->Read_Package_Buffer[1]&0x1f;
  CRC_Check = 0;
  for(int i=0;i<pp->Read_Package_Length-1;i++)
  {
    CRC_Check += pp->Read_Package_Buffer[i];
  }
  CRC_Check ^= pp->Read_Package_Buffer[pp->Read_Package_Length-1];
  CRC_Check &= 0x7f;
  
  if(CRC_Check!= 0){
      post("CRC Error\n");
      return CRC_Error;
  }
  switch(ReceivedFunction_Code){
        case Is_AbsPos32:
        case Is_TrqCurrent:
        case Is_GearNumber:
        case Is_Config :
        case Is_Status :
            value = Cal_SignValue(pp->Read_Package_Buffer);
            break;
        case Is_MainGain:
        case Is_SpeedGain:
        case Is_IntGain:
        case Is_TrqCons:
        case Is_HighSpeed:
        case Is_HighAccel:
        case Is_Drive_ID:
        case Is_PosOn_Range:
            value = Cal_UnsignedValue(pp->Read_Package_Buffer);
            break;
        default:
            value = Cal_SignValue(pp->Read_Package_Buffer);
  }
  //
    if (pp->Drive_Read_Code == Is_AbsPos32) {
        pp->ReportPositionPtr(value,pp->hook);
    } else {
        post("Axis: %d, %s (%d): Value: %ld\n",
             ID,
             ParameterName(ReceivedFunction_Code),
             ReceivedFunction_Code,
             value);
    }
  pp->Drive_Read_Axis_ID = ID;
  pp->Drive_Read_Code = (unsigned char)ReceivedFunction_Code;
  pp->Drive_Read_Value = value;
  return Complete_Success;
}

/*
bool printStatusByte(unsigned char statusByte) {
    bool FatalError = false;
    printf("Motor Status\n");
    if (statusByte & 1) { // bit 0
        printf("\tMotor In position\n");
    } else {
        printf("\tMotor Out of Position\n");
    }
    
    if (statusByte & 2) { // bit 1
        printf("\tMotor Free/Disengaged\n");
    } else {
        printf("\tMotor Active/Enagaged\n");
    }
    
    if (statusByte & 28) { // bits 2,3,4
        printf("\tALARM: ");
        int alarmCode = (statusByte & 28) >> 2;
        switch (alarmCode) {
            case 1:
                printf("Lost Phase, |Pset - Pmotor|>8192(steps), 180(deg)\n");
                FatalError = true;
                break;
            case 2:
                printf("Over Current\n");
                FatalError = true;
                break;
            case 3:
                printf("Over Heat or Over Power\n");
                FatalError = true;
                break;
            case 4:
                printf("CRC Error Report, Command not Accepted\n");
                break;
            default:
                printf("Unkown Error\n");
        }
    } else {
        //printf("No Alarm");
    }
    
    if ((statusByte & 32) == 0) { // bit 5
        printf("\tWaiting for next S-curve,lieanr,circular motion\n");
    } else {
        printf("\tBUSY with current S-curve,lieanr,circular motion\n");
    }
    
    Boolean pin2JP3 = (statusByte & 64);  // bit 6
    printf("\tCNC Zero Position (PIN 2 of JP3): %s\n", (pin2JP3) ? "HIGH" : "LOW");
    return FatalError;
}
*/
 
/*Get data with sign - long*/
long Cal_SignValue(unsigned char One_Package[8])
{
  char Package_Length,OneChar;
  int i;
  long Lcmd;
  OneChar = One_Package[1];
  OneChar = OneChar>>5;
  OneChar = OneChar&0x03;
  Package_Length = 4 + OneChar;
  OneChar = One_Package[2]; /*First byte 0x7f, bit 6 reprents sign */
  OneChar = OneChar << 1;
  Lcmd = (long)OneChar; /* Sign extended to 32bits */
  Lcmd = Lcmd >> 1;
  for(i=3;i<Package_Length-1;i++)
  {
    OneChar = One_Package[i];
    OneChar &= 0x7f;
    Lcmd = Lcmd<<7;
    Lcmd += OneChar;
  }
  return(Lcmd); /* Lcmd : -2^27 ~ 2^27 - 1 */
}

/*Get data with out  - unsigned */
unsigned int Cal_UnsignedValue(unsigned char One_Package[8])
{
    char Package_Length,OneChar;
    int i;
    unsigned int ret;
    OneChar = One_Package[1];
    OneChar = OneChar>>5;
    OneChar = OneChar&0x03;
    Package_Length = 4 + OneChar;
    OneChar = One_Package[2] & 0x7f; // only 7 bits are of data bits
    ret = (unsigned int)OneChar;
    for(i=3;i<Package_Length-1;i++)
    {
        OneChar = One_Package[i];
        OneChar &= 0x7f;
        ret = ret<<7;
        ret += OneChar;
    }
    return(ret); /* ret : 0 ~ 127 */
}


// ***************** Every Robot Instruction ******************
// Send a package with a function by Global_Func
// Displacement: -2^27 ~ 2^27 - 1
// Note: in the description of RS232 communication protocol above (Section 7), the last byte of packet is
// always B0, but in the code of below, the first byte is always B0.
//

void Send_Package(DmmProtocolState_t* pp,unsigned char func, char ID , long Displacement)
{
  pp->ProtocolError = false;
    
  unsigned char B[8],Package_Length,Function_Code;
  long TempLong;
  B[1] = B[2] = B[3] = B[4] = B[5] = (unsigned char)0x80;
  B[0] = ID&0x7f;
  Function_Code = func & 0x1f;
  TempLong = Displacement & 0x0fffffff; //Max 28bits
  B[5] += (unsigned char)TempLong&0x0000007f;
  TempLong = TempLong>>7;
  B[4] += (unsigned char)TempLong&0x0000007f;
  TempLong = TempLong>>7;
  B[3] += (unsigned char)TempLong&0x0000007f;
  TempLong = TempLong>>7;
  B[2] += (unsigned char)TempLong&0x0000007f;
  Package_Length = 7;
  TempLong = Displacement;
  TempLong = TempLong >> 20;
  if(( TempLong == 0x00000000) || ( TempLong == 0xffffffff))
  {//Three byte data
    B[2] = B[3];
    B[3] = B[4];
    B[4] = B[5];
    Package_Length = 6;
  }
  TempLong = Displacement;
  TempLong = TempLong >> 13;
  if(( TempLong == 0x00000000) || ( TempLong == 0xffffffff))
  {//Two byte data
    B[2] = B[3];
    B[3] = B[4];
    Package_Length = 5;
  }
  TempLong = Displacement;
  TempLong = TempLong >> 6;
  if(( TempLong == 0x00000000) || ( TempLong == 0xffffffff))
  {//One byte data
    B[2] = B[3];
    Package_Length = 4;
  }
  B[1] += (Package_Length-4)*32 + Function_Code;
  Make_CRC_Send(pp, Package_Length,B);
}


void Make_CRC_Send(DmmProtocolState_t* pp, unsigned char Plength,unsigned char B[8])
{
  unsigned char Error_Check = 0;
  for(int i=0;i<Plength-1;i++) {
    pp->SerialWritePtr(B[i],pp->hook);
    Error_Check += B[i];
  }
  Error_Check = Error_Check|0x80;
  pp->SerialWritePtr(Error_Check, pp->hook);
}


/* - Reading
void ReadMotorTorqueCurrent(char AxisID)  {
    
  // Below are the codes for reading the motor torque current
  //Read motor torque current
  Send_Package(General_Read, AxisID , Is_TrqCurrent);
  //Function code is General_Read, but one byte data is : Is_TrqCurrent
  //Then the drive will return a packet, Function code is Is_TrqCurrent
  //and the data is 16bits Motor torque current.
  while(ProtocolError == In_Progress) {
    ReadPackage();
  }
}

 */

void MoveMotorToAbsolutePosition32(DmmProtocolState_t* pp, char Axis_Num,long Pos32)
{
  Send_Package(pp,Go_Absolute_Pos, Axis_Num, Pos32);
}

void MoveMotorConstantRotation(DmmProtocolState_t* pp, char Axis_Num,long r) {
    // TODO set Limits for 3 byte value of r
    Send_Package(pp, Turn_ConstSpeed, Axis_Num, r);
}

void ResetOrgin(DmmProtocolState_t* pp, char Axis_Num) {
    Send_Package(pp, Set_Origin, Axis_Num, 0); // 0: Dummy Data
}

void SetMaxSpeed(DmmProtocolState_t* pp,char Axis_Num, int maxSpeed) {
  long m = MAX(1,MIN(127,maxSpeed));
  Send_Package(pp,Set_HighSpeed, Axis_Num, m);
}

void SetMaxAccel(DmmProtocolState_t* pp, char Axis_Num, int maxAccel) {
    long m = MAX( 1, MIN( 127, maxAccel));
    Send_Package(pp, Set_HighAccel, Axis_Num, m);
}

void SetMainGain(DmmProtocolState_t* pp, char Axis_Num, int gain) {
    long l = MAX( 1, MIN( 127, gain));
    Send_Package(pp, Set_MainGain, Axis_Num, l);
}

void SetSpeedGain(DmmProtocolState_t* pp,char Axis_Num, long gain) {
    long l = MAX( 1, MIN( 127, gain));
    Send_Package(pp, Set_SpeedGain, Axis_Num, l);
}

void SetIntGain(DmmProtocolState_t* pp, char Axis_Num, long gain) {
    long l = MAX(1, MIN(127, gain));
    Send_Package(pp, Set_IntGain, Axis_Num, l);
}

void MotorDisengage(DmmProtocolState_t* pp, char Axis_Num, unsigned char curConfig) {
    // Free Shaft
    Send_Package(pp, Set_Drive_Config, Axis_Num, curConfig | Config_Bit_MOTOR_DRIVE);
}

void MotorEngage(DmmProtocolState_t* pp, char Axis_Num, unsigned char curConfig) {
    Send_Package(pp, Set_Drive_Config, Axis_Num, curConfig & !Config_Bit_MOTOR_DRIVE);
}

/* TODO - Reading
void ReadMainGain(DmmProtocolState_t* pp, char Axis_Num) {
  Send_Package(pp, Read_MainGain, Axis_Num, Is_MainGain);
  pp->ProtocolError = In_Progress;
  while(pp->ProtocolError == In_Progress) {
      ReadPackage();
  }
}

long ReadParamer(DmmProtocolState_t *pp, char queryParam, char Axis_Num) {
    pp->ProtocolError = In_Progress;
    pp->Drive_Read_Value = -1;
    pp->Drive_Read_Code = -1;
    Send_Package(pp, queryParam, Axis_Num, 0 ); // 0 is a dummy Data Value
    while(pp->ProtocolError == In_Progress) {
        ReadPackage();
    }
//    printf("%s: %ld\n",ParameterName(Drive_Read_Code), Drive_Read_Value);
    if (pp->ProtocolError == Complete_Success) {
        return pp->Drive_Read_Code;
    } else {
        return LONG_MIN;
    }
}

*/
void ReadMotorPosition32(DmmProtocolState_t *pp, char Axis)
{ // Below are the codes for reading the motor shaft 32bits absolute position
    //Read motor 32bits position
    pp->ProtocolError = In_Progress;
    Send_Package(pp, General_Read, Axis , Is_AbsPos32);
}

/*
int testMotor(void)
{
    const char Axis_Num = 0;
    unsigned char statusByte = -1;
    printf("DMM Test Motor\n\n");
    
    long result = ReadParamer(Read_Drive_Status, Axis_Num);
    bool FatalError = true;
    if (result != LONG_MIN ) {
        statusByte = (unsigned char)(result & 0x7f);
        FatalError = printStatusByte(Drive_Read_Value);
    }
    
    if (FatalError || result == LONG_MIN) {
        return statusByte;
        // STOP  MOTOR PISSED!
    }

    long delta = 5; //100;
    long center = 0;
    int delay_ms = 1000;

    long config = ReadParamer(Read_Drive_Config, Axis_Num);
    if (configByte >= 0 && config <= 0x7f) {
        configByte = (unsigned char)config;
    }
    
    
    
    // limited "Set" writes to firmware dont' use in loopDmmDriver_SerialPort_h
    //SetMainGain(Axis_Num, 60); //[15]// [14~20~40(loud)]relative to load, increase as load increases
    ReadParamer(Read_MainGain, Axis_Num);  ReadMainGain(Axis_Num); // Param is MotorID Main Gain stored in MainGain_Read variable
    // SetSpeedGain(Axis_Num, 127); // [127] higher : less dynamic movements
    ReadParamer(Read_SpeedGain, Axis_Num);
    ReadMotorPosition32(Axis_Num);
    
   // SetIntGain(Axis_Num, 1); // higher for rigid system, lower for loose system (outside disturbance)
                             // lag in feedback from encoder
    
    ReadParamer(Read_IntGain, Axis_Num);
    ReadParamer(Read_Pos_OnRange, Axis_Num);
    ReadParamer(Read_GearNumber, Axis_Num); // 500
    ReadParamer(Read_Drive_Config, Axis_Num);
    
#if false
    
    // reset Origin to Down/Reset Position
    ResetOrgin(Axis_Num);
    //return 0;
    
    // These are not remembered
    SetMaxSpeed(Axis_Num, 10); // 1
    SetMaxAccel(Axis_Num, 16); // 4

    
    //MoveMotorToAbsolutePosition32(Axis_Num,0);
    
    
    printf("\n");
    SetMaxSpeed(Axis_Num, 2); // 1
    SetMaxAccel(Axis_Num, 6); // 6
    
    MoveMotorConstantRotation(0,0);
    delay(50);
    return 0;
    for(;;) {
        if ((center % 3000) == 0) {
            Send_Package(General_Read, 0 , Is_AbsPos32);
            delta = delta * -1;
        }
        ReadPackage();
        delay(10);
     

//        SetMaxSpeed(Axis_Num, 1); // 1
//        SetMaxAccel(Axis_Num, 6); // 4
        
        center += delta;
        MoveMotorToAbsolutePosition32(Axis_Num, center);
    }
    
    MoveMotorToAbsolutePosition32(Axis_Num, center + delta);
//    ReadMotorTorqueCurrent(Axis_Num); //Motor torque current stored in MotorTorqueCurrent variable
    delay(delay_ms);

    MoveMotorToAbsolutePosition32(Axis_Num, center);
//    ReadMotorTorqueCurrent(Axis_Num); //Motor torque current stored in MotorTorqueCurrent variable
    delay(delay_ms);
 //   ReadMotorPosition32(Axis_Num); //Motor absolute position stored in Motor_Pos32 variable
    
    MoveMotorToAbsolutePosition32(Axis_Num, center + -1 * delta);
//    ReadMotorTorqueCurrent(Axis_Num); //Motor torque current stored in MotorTorqueCurrent variable
    delay(delay_ms);
 //   ReadMotorPosition32(Axis_Num); //Motor absolute position stored in Motor_Pos32 variable

    MoveMotorToAbsolutePosition32(Axis_Num, center);
        //   ReadMotorTorqueCurrent(Axis_Num); //Motor torque current stored in MotorTorqueCurrent variable
    delay(delay_ms);

        
    //}
    
#endif
    
    MotorDisengage(Axis_Num,configByte);
    delay(50);
    return 0;
}


int main() {
    int ret = 0;
    if (openSerial("/dev/cu.usbserial") == EX_OK) {
         ret = testMotor();
        closeSerial();
    }
    return ret;
}
*/