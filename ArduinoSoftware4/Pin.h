const byte RStepPin=54;
const byte RDirPin=55;
const byte REnablePin=38;
const byte REndstopPin=3;
const byte AStepPin=60;
const byte ADirPin=61;
const byte AEnablePin=56;
const byte AEndstopPin=2;
const byte ZStepPin=46;
const byte ZDirPin=48;
const byte ZEnablePin=62;
const byte ZEndstopPin=18;
const byte EStepPin=26;
const byte EDirPin=28;
const byte EEnablePin=24;
const byte E0StepPin=36;
const byte E0DirPin=34;
const byte E0EnablePin=30;

const byte HotendThermistorPin=67;
const byte HotendHeaterPin=8;
const byte HotendFanPin=10;
const byte PrintFanPin=9;

const byte ROnPin=17;
const byte AOnPin=23;
const byte ZOnPin=25;
const byte EOnPin=27;
const byte MotorOnGroundPin=16;


void SetPins(){
  pinMode(RStepPin,OUTPUT);
  pinMode(RDirPin,OUTPUT);
  pinMode(REnablePin,OUTPUT);
  pinMode(REndstopPin,INPUT);
  pinMode(AStepPin,OUTPUT);
  pinMode(ADirPin,OUTPUT);
  pinMode(AEnablePin,OUTPUT);
  pinMode(AEndstopPin,INPUT);
  pinMode(ZStepPin,OUTPUT);
  pinMode(ZDirPin,OUTPUT);
  pinMode(ZEnablePin,OUTPUT);
  pinMode(ZEndstopPin,INPUT);
  pinMode(EStepPin,OUTPUT);
  pinMode(EDirPin,OUTPUT);
  pinMode(EEnablePin,OUTPUT);

  pinMode(HotendThermistorPin,INPUT);
  pinMode(HotendHeaterPin,OUTPUT);
  pinMode(HotendFanPin,OUTPUT);
  pinMode(PrintFanPin,OUTPUT);

  pinMode(ROnPin,OUTPUT);
  pinMode(AOnPin,OUTPUT);
  pinMode(ZOnPin,OUTPUT);
  pinMode(EOnPin,OUTPUT);
  pinMode(MotorOnGroundPin,OUTPUT);
}
void ResetPins(){
  digitalWrite(RStepPin,LOW);
  digitalWrite(RDirPin,HIGH);
  digitalWrite(REnablePin,HIGH);
  digitalWrite(AStepPin,LOW);
  digitalWrite(ADirPin,HIGH);
  digitalWrite(AEnablePin,HIGH);
  digitalWrite(ZStepPin,LOW);
  digitalWrite(ZDirPin,HIGH);
  digitalWrite(ZEnablePin,HIGH);
  digitalWrite(EStepPin,LOW);
  digitalWrite(EDirPin,HIGH);
  digitalWrite(EEnablePin,HIGH);

  digitalWrite(HotendHeaterPin,LOW);
  digitalWrite(HotendFanPin,LOW);
  digitalWrite(PrintFanPin,LOW);

  digitalWrite(ROnPin,LOW);
  digitalWrite(AOnPin,LOW);
  digitalWrite(ZOnPin,LOW);
  digitalWrite(EOnPin,LOW);
  digitalWrite(MotorOnGroundPin,LOW);
}
