

//PrinterSettings
const float PivotToTangent=392;
const float NozzleToR=26;
const float PivotX=243;
const int RStepsPerMM=25;
const int AStepsPerMM=25;
const int ZStepsPerMM=100;
const float EStepsPerMM=24.44;
const byte RMicrostep=4;
const byte AMicrostep=4;
const byte ZMicrostep=4;
const byte EMicrostep=8;
const int REndstopPosition=-4700; //steps endstop coordinates
const int AEndstopPosition=-5500;
const int ZEndstopPosition=25800;
const float XHome=0; //cartesian home coordinates
const float YHome=0;
const float ZHome=60;

const float CartToStepK=100; //default scale between mm and steps
const float CartToExtrK=0.0188; //default scale between XYmm and Emm
const float MinVend=CartToStepK; //minimum end velocity
const float VtargToVZDefaultK=0.1; //speed related to feedrate, when is not bound by RA movement profile
const float StandardFeedRate=1500;




//SYSTEM VARIABLES
long Time=0; //millis time
const int LoopDelay=1; //milllis
boolean DepthLog=false; //if should echo back to the server in depth information
boolean DepthLogPlus=false;

//serial communication and g code buffer
String SerialMsg="";
boolean SerialComplete=true; //trigger for start and end of serial recording
const byte BufferSize=10;
String Buffer[BufferSize]; //buffer of the serial messages received
byte BufferHead=0; //index of the first element
byte BufferTail=0; //index+1 of the last element, buffer length=tail-head

boolean FinishedCode=false; //if the current Gcode has been finished

const byte MotionBufferSize=24;
byte MotionBufferFilled=0;

//timer to check if buffer has refilled, in wich case call nextcode
int NextCodeTimert=0;
const int NextCodeTimerT=2000;
boolean AutoNextCode=true;

boolean Ron=false; boolean Aon=false; //reflects the state of the motor
boolean Zon=false; boolean Eon=false;
boolean Ractive=false; boolean Aactive=false; //if the motor will is/hasbeen/isgonnabe used
boolean Zactive=false; boolean Eactive=false;

//STATE VARIABLES
//global variables for queuing and running MotionProfiles
boolean MotionCode=false;
boolean NextPresent=false;
//current coordinates
float XCart=0; float YCart=0;
float ZCart=0; float ECart=0;
int RStep=0; int AStep=0;
word ZStep=0; float EStep=0;
//target coordinates
float XtargCart=0; float YtargCart=0;
float ZtargCart=0; float EtargCart=0;
int RtargStep=0; int AtargStep=0;
word ZtargStep=0; float EtargStep=0;
//other
float XnexttargCart=0; float YnexttargCart=0;
int RnexttargStep=0; int AnexttargStep=0;
float dStep=0; float dnextStep=0;
//to create the MotionProfile object, relevant to the last profile added
float vtarg=0; float vnxttarg=0; float vstart=0; float vend=0;
word dr=0; word da=0; word dz=0; word de=0;

//RA motion profile global variables, live
const float AccelCart=50; //cartesian accelleration
const float AccelStep=CartToStepK*AccelCart; //step acceleration, scaled from AccelCart
float QueueProfileLastVend=0; //Vend of the last added motionprofile to the queue
//Go Target algorithm, variables updated every gotarget
unsigned long Tstart=0UL; //value at motion start (micros)
unsigned long Tstate=0UL; //time passed since motion start (micros)
boolean Rdir=true; boolean Adir=true; boolean Zdir=true; boolean Edir=true;
unsigned long RSim=0UL; unsigned long ASim=0UL; unsigned long ZSim=0UL; unsigned long ESim=0UL;
unsigned long dRSim=0UL; unsigned long dASim=0UL; unsigned long dZSim=0UL; unsigned long dESim=0UL;
byte MotionPhase=5; //in which phase the motion is currently in (1: accel, 2: vtarg, 3: decel, 4: vend, 5: finished)
const word dVT=1000; unsigned long dVt=0UL+dVT; //update speeds timer
const word GoSimT=100; unsigned long GoSimt=0UL; //go target simulation timer
const byte GoT=49; //go target loop delay
boolean Go=false; //go target while loop condition
const unsigned long GoSimScale=(unsigned long)(65000.0*1000000000000.0/(AccelStep*dVT*GoSimT)); //simulation S, V, A are scaled by this factor, needed to use long instead of float
const float dSimStandard=GoSimT/1000000.0*GoSimScale; //factor out constants for faster calculation during profile creation
const float ddSimStandard=((AccelStep*dVT*GoSimT)/1000000000000.0)*GoSimScale; 

//other state variables
float FeedRate=StandardFeedRate; //speed factor(mm/min)
boolean AbsolutePositioning=true; //absolute/relative mode for position
boolean AbsoluteExtrusion=false; //absolute/relative mode for extruder


//Gcode received attributes, modified only directly by Gcodes
float C=0; //custom comand
float X=0; //received coordinates
float Y=0;
float Z=0;
float E=0;
float F=0; //feedrate (mm/min)
float S=0; //tool target temperature (°C)


#define sgn(x) ((x) < 0 ? -1 : 1) //sgn function
