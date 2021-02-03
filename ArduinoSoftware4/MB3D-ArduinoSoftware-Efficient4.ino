
#include "Pin.h"
#include "GlobalVariables.h"

#include <digitalWriteFast.h>

#include "ThermistorData.h"
#include "ThermalManagement.h"



//-----------------------------SETUP AND LOOP--------------------------

void setup() {

  Serial.begin(115200);
  SerialMsg.reserve(100);
  for(int i=0; i<BufferSize; i++){ Buffer[i]="/"; }

  SetPins();
  ResetPins();
 
}



void loop() {
  Time=millis();

  if(AutoNextCode){
    if(NextCodeTimert<Time){
      NextCodeTimert=Time+NextCodeTimerT;
      if(BufferHead!=BufferTail){
        AutoNextCode=false;
        Serial.print("autonext}");
        NextCode();
      }
    }
  }

  ThermalTimerUpdate();

  if(FinishedCode){
    CheckSerial();
    if(MotionCode && !NextPresent){
      RunMotionQueue();
    }
    FinishCode();
  }
  
  
  delay(LoopDelay);
}

//-----------------------------SETUP AND LOOP--------------------------





//---------------------COMMUNICATION ALGORITHM--------------------------

void serialEvent(){
  delay(100);
  CheckSerial();
}
void CheckSerial(){
  Time=millis();
  NextCodeTimert=Time+NextCodeTimerT;
  char c=' ';
  while(Serial.available()){
    c=Serial.read();
    if(c=='{'){
      SerialComplete=false;
      SerialMsg="";
    }else if(c=='}'){
      if(!SerialComplete){
        if(SerialMsg.equals("report")){
          PrintReport();
        }else{
          if(DepthLog){ Serial.print("echo/adding buffer: "+SerialMsg+"}"); }
          Serial.print("buf}");
          Buffer[BufferTail]=SerialMsg;
          BufferTail=(BufferTail+1==BufferSize)?0:(BufferTail+1);
        }
      }
      SerialComplete=true;
      SerialMsg="";
    }else if(!SerialComplete){
      SerialMsg+=c;
    }
  }
}

//finish the current code
void FinishCode(){
  MotionCode=false;
  FinishedCode=false;
  Serial.print("fin}");
  Buffer[BufferHead]="/";
  BufferHead=(BufferHead+1==BufferSize)?0:(BufferHead+1);
  if(BufferHead!=BufferTail){
    NextCode();
  }else{
    AutoNextCode=true;
    NextCodeTimert=0;
  }
}
//call next code in the buffer
void NextCode(){
  if(DepthLog){ Serial.print("echo/executing code: "+Buffer[BufferHead]+"}"); }
  GcodeCall(Buffer[BufferHead]);
}
//prints various information about the printer state
void PrintReport(){
  Serial.print("echo/report: ------------------------\n");
  Serial.print("buffer: ");
  for(int i=0; i<BufferSize; i++){
    if(i==BufferHead){Serial.print("H:");}
    if(i==BufferTail){Serial.print("T:");}
    Serial.print(String(Buffer[i])+"  ");
  }
  Serial.print("end of report ----------------------}");
}

//---------------------COMMUNICATION ALGORITHM--------------------------







//-----------------MOTOR COMANDS---------------------
void TurnOffMotors(){
  if(Ractive){ Ractive=false; Ron=false; digitalWrite(REnablePin,HIGH); digitalWrite(ROnPin,LOW); }
  if(Aactive){ Aactive=false; Aon=false; digitalWrite(AEnablePin,HIGH); digitalWrite(AOnPin,LOW); }
  if(Zactive){ Zactive=false; Zon=false; digitalWrite(ZEnablePin,HIGH); digitalWrite(ZOnPin,LOW); }
  if(Eactive){ Eactive=false; Eon=false; digitalWrite(EEnablePin,HIGH); digitalWrite(EOnPin,LOW); }
}
void TurnOnMotors(){
  if(Ractive){ Ron=true; digitalWrite(REnablePin,LOW); digitalWrite(ROnPin,HIGH); }
  if(Aactive){ Aon=true; digitalWrite(AEnablePin,LOW); digitalWrite(AOnPin,HIGH); }
  if(Zactive){ Zon=true; digitalWrite(ZEnablePin,LOW); digitalWrite(ZOnPin,HIGH); }
  if(Eactive){ Eon=true; digitalWrite(EEnablePin,LOW); digitalWrite(EOnPin,HIGH); }
}

//cartesian to step conversion
int CartToR=0; int CartToA=0;
void UpdateCartToRA(float x, float y){
  float a2=(y+NozzleToR)/(PivotX-x);
  float r1=sqrt(sq(PivotX-x)+y*(y+NozzleToR+NozzleToR));
  float a1=PivotToTangent*(a2*r1-NozzleToR)/(r1+a2*NozzleToR);
  CartToR=int((PivotX-r1)*RStepsPerMM*RMicrostep);
  CartToA=int(a1*AStepsPerMM*AMicrostep);
  //CartToR=int(x*CartToStepK);
  //CartToA=int(y*CartToStepK);
}
word CartToZ(float z){ return (word(max(0,z*ZStepsPerMM*ZMicrostep))); }
float CartToE(float e){ return e*EStepsPerMM*EMicrostep; }
//step to cartesian conversion
float StepToX=0; float StepToY=0;
void UpdateStepToXY(int r, int a){
  float r1=(PivotX-r/(RStepsPerMM*RMicrostep)); float a1=a/(AStepsPerMM*AMicrostep);
  float r2=sqrt(sq(NozzleToR)+sq(r1)); float a2=(a1+PivotToTangent*NozzleToR/r1)/(1-a1*NozzleToR/(PivotToTangent*r1));
  float c=sqrt(sq(PivotToTangent)+sq(a2));
  StepToX=r2*PivotToTangent/c-PivotX;
  StepToY=r2*a2/c-NozzleToR;
}
float StepToZ(word z){
  return float(z)/(ZStepsPerMM*ZMicrostep);
}
float StepToE(float e){
  return e/(EStepsPerMM*EMicrostep);
}


void FindEndstop(int n){
  if(DepthLog){
    Serial.print("echo/Searching endstop "+String(n)+"}");
  }
  int dt=int(500000.0/(StandardFeedRate/60*CartToStepK));
  boolean toggle=false;
  int stp=-1;
  int endstp=-1;
  switch (n){
    case 1: Ractive=true; digitalWrite(RDirPin,LOW); Rdir=false; stp=RStepPin; endstp=REndstopPin; RStep=REndstopPosition; RtargStep=RStep; break;
    case 2: Aactive=true; digitalWrite(ADirPin,LOW); Adir=false; stp=AStepPin; endstp=AEndstopPin; AStep=AEndstopPosition; AtargStep=AStep; break;
    case 3: Zactive=true; digitalWrite(ZDirPin,HIGH); Zdir=true; stp=ZStepPin; endstp=ZEndstopPin; ZStep=ZEndstopPosition; ZtargStep=ZStep; break;
  }
  TurnOnMotors();
  while(digitalRead(endstp)){
    toggle=!toggle; digitalWrite(stp,toggle);
    delayMicroseconds(dt);
  }
  TurnOffMotors();
  
  UpdateStepToXY(RStep,AStep);
  XCart=StepToX; YCart=StepToY;
  ZCart=StepToZ(ZStep); ECart=StepToE(EStep);
  XtargCart=XCart; YtargCart=YCart; ZtargCart=ZCart; EtargCart=ECart;
}



//-----------------MOTOR COMANDS---------------------








//----------------------MOTION PROFILE-----------------------------

//Motion Profile object
class MotionProfile{
  word dR=0; word dA=0; word dZ=0; word dE=0;
  boolean rdir=true; boolean adir=true; boolean zdir=true; boolean edir=true;
  //variables to calculate
  unsigned long dRSimstart=0UL; unsigned long dASimstart=0UL; unsigned long dESimstart=0UL; unsigned long dZSimstart=0UL;
  word ddRSim=0UL; word ddASim=0UL; word ddESim=0UL; //velocity change over dVT time, overflow warning
  unsigned long MotionT=0UL; //micros
  unsigned long ddVT1=0UL; unsigned long ddVT2=0UL; //micros
    
  public:
    MotionProfile(){
    }

    void MotionSetup(float Vtarg, float Vstart, float Vend,
        word dr, word da, word dz, word de, boolean rd, boolean ad, boolean zd, boolean ed){

      rdir=rd; adir=ad; zdir=zd; edir=ed;
      dR=dr; dA=da; dZ=dz; dE=de;

      float StepRatioR=0; float StepRatioA=0; float StepRatioE=0;
      
      float phase1_t=0; float phase1_2AS=0;
      float phase2_t=0; float phase2_2AS=0;
      float phase3_t=0; float phase3_2AS=0;
      float Mot2AS=2*AccelStep*dStep;
      float T=0;
      if(dR!=0 ||dA!=0){
        StepRatioR=dR/dStep; StepRatioA=dA/dStep; StepRatioE=dE/dStep;
        
        float V1=min(Vstart,Vend);
        float V2=max(Vstart,Vend);
        phase1_t=(V2-V1)/AccelStep;
        phase1_2AS=sq(V2)-sq(V1);
        //if can't accelerate in time to V2
        if(Mot2AS<phase1_2AS){
          phase1_2AS=Mot2AS;
          phase1_t=((Vend<Vstart)?-1:1)*(sqrt(sq(Vstart)+((Vend<Vstart)?-1:1)*phase1_2AS)-Vstart)/AccelStep;
          T=phase1_t;
        }else{
          phase2_t=(Vtarg-V2)/AccelStep;
          phase2_2AS=2*(sq(Vtarg)-sq(V2));
          //if can't accelerate in time to Vtarg
          if(Mot2AS-phase1_2AS<phase2_2AS){
            phase2_2AS=Mot2AS-phase1_2AS;
            phase2_t=(sqrt(sq(V2)+phase2_2AS/2)-V2)/AccelStep;
            T=phase1_t+2*phase2_t;
          }else{
            phase3_2AS=Mot2AS-phase1_2AS-phase2_2AS;
            phase3_t=phase3_2AS/(AccelStep*4*Vtarg);
            T=phase1_t+2*phase2_t+2*phase3_t;
          }
        }
        
        //velocity change over dVT time during accel/decel
        ddRSim=StepRatioR*ddSimStandard+1; ddASim=StepRatioA*ddSimStandard+1; ddESim=StepRatioE*ddSimStandard+1;
         
        //position change over GoSimT time at start
        dRSimstart=Vstart*StepRatioR*dSimStandard+0.5*ddRSim+0.5; dASimstart=Vstart*StepRatioA*dSimStandard+0.5*ddASim+0.5; dESimstart=Vstart*StepRatioE*dSimStandard+0.5*ddESim+0.5; dZSimstart=dZ/T*dSimStandard+1;
        
        if(Vend>=Vstart){ ddVT1=((phase1_t+phase2_t)*1000000); ddVT2=((phase1_t+phase2_t+2*phase3_t)*1000000); }
        else{ ddVT1=(phase2_t*1000000); ddVT2=((phase2_t+2*phase3_t)*1000000); }
        
      }else{
        dRSimstart=0UL; dASimstart=0UL;
        ddRSim=0; ddASim=0; ddESim=0;
        ddVT1=0UL; ddVT2=0UL; 
        
        //Z > E motion profile priority
        if(dZ!=0){
          dZSimstart=Vtarg*VtargToVZDefaultK*dSimStandard+1;
          T=dZ/(Vtarg*VtargToVZDefaultK);
          dESimstart=dE/T*dSimStandard+1;
        }
        else if(dE!=0){
          dESimstart=Vtarg*CartToExtrK*dSimStandard+1;
          T=dE/(Vtarg*CartToExtrK);
        }
        
      }
      MotionT=T*1000000;

      if(DepthLog){
        Serial.print("echo/calculated motion profile, d: "+String(dR)+", "+String(dA)+", "+String(dZ)+", "+String(dE)+"\n"
        +"Vstart: "+String(Vstart)+", Vtarg: "+String(Vtarg)+", Vend: "+String(Vend)+"\n");
        if(DepthLogPlus){
          Serial.print("dStep: "+String(dStep)+", phase S: "+String(phase1_2AS/(2*AccelStep))+", "+String(phase2_2AS/(2*AccelStep))+", "+String(phase3_2AS/(2*AccelStep))+"\n"
          +"MotionT: "+String(MotionT)+", phase t: "+String(phase1_t)+", "+String(phase2_t)+", "+String(phase3_t));
        }else{
          Serial.print("MotionT: "+String(MotionT));
        }
        Serial.print("\ndSimstart: "+String(dRSimstart)+", "+String(dASimstart)+", "+String(dZSimstart)+", "+String(dESimstart)+"\n"
        +"ddSim: "+String(ddRSim)+", "+String(ddASim)+", "+String(ddESim)+"}");
      }
    }
    //prepare the relevant motors before runmotionqueue really begins
    void Prepare(){
      if(dR!=0){Ractive=true;}
      if(dA!=0){Aactive=true;}
      if(dZ!=0){Zactive=true;}
      if(dE!=0){Eactive=true;}
    }

    void PrintPhaseLog(){
      Serial.print("echo/T: "+String(Tstate)+", GoSimt: "+String(GoSimt)+", phase "+String(MotionPhase));
      if(DepthLogPlus){
        Serial.print("\nd: "+String(dR)+", "+String(dA)+", "+String(dZ)+", "+String(dE)+"\n"
        +"dSim: "+String(dRSim)+", "+String(dASim)+", "+String(dZSim)+", "+String(dESim)+"\n"
        +"Sim: "+String(RSim)+", "+String(ASim)+", "+String(ZSim)+", "+String(ESim));
      }
      Serial.print("}");
    }
    
    void GoTarget(){
      MotionPhase=4;
      if(dR!=0){ dRSim=dRSimstart; RSim=0UL; if(rdir){digitalWriteFast(RDirPin,HIGH);}else{digitalWriteFast(RDirPin,LOW);} MotionPhase=1; }
      if(dA!=0){ dASim=dASimstart; ASim=0UL; if(adir){digitalWriteFast(ADirPin,HIGH);}else{digitalWriteFast(ADirPin,LOW);} MotionPhase=1; }
      if(dZ!=0){ dZSim=dZSimstart; ZSim=0UL; if(zdir){digitalWriteFast(ZDirPin,HIGH);}else{digitalWriteFast(ZDirPin,LOW);}}
      if(dE!=0){ dESim=dESimstart; ESim=0UL; if(edir){digitalWriteFast(EDirPin,HIGH);}else{digitalWriteFast(EDirPin,LOW);}}

      if(DepthLog){
        /*Serial.print("echo/starting motion, MotionT: "+String(MotionT)+", phase "+String(MotionPhase)+"\n"
        +"ddVT1: "+String(ddVT1)+", ddVT2: "+String(ddVT2));
        if(DepthLogPlus){
          Serial.print("\nd: "+String(dR)+", "+String(dA)+", "+String(dZ)+", "+String(dE)+"\n"
          +"dSim: "+String(dRSim)+", "+String(dASim)+", "+String(dZSim)+", "+String(dESim)+"\n"
          +"Sim: "+String(RSim)+", "+String(ASim)+", "+String(ZSim)+", "+String(ESim));
        }
        Serial.print("}");*/
        Serial.print("echo/starting motion, MotionT: "+String(MotionT)+", phase "+String(MotionPhase)
        +"\nd: "+String(dR)+", "+String(dA)+", "+String(dZ)+", "+String(dE)+"}");
      }

      Go=true;
      while(Go){
        Tstate=micros()-Tstart;
        if(Tstate>GoSimt){
          Go=false;
          GoSimt+=GoSimT;

          if(Tstate>dVt){
            dVt+=dVT;
            if(Tstate>MotionT){
              MotionPhase=4; if(DepthLog){PrintPhaseLog();}
              dVt=1000000000UL;
            }else if(Tstate>ddVT2){
              if(MotionPhase==2){ MotionPhase=3; if(DepthLog){PrintPhaseLog();} }
              dRSim-=ddRSim; dASim-=ddASim; dESim-=ddESim;
            }else if(Tstate<ddVT1){
              dRSim+=ddRSim; dASim+=ddASim; dESim+=ddESim;
            }else if(MotionPhase==1){
              MotionPhase=2; if(DepthLog){PrintPhaseLog();}
            }
          }
          
          if(dR!=0){ Go=true; RSim+=dRSim;
            if(RSim>GoSimScale){ digitalWriteFast(RStepPin,HIGH); RSim-=GoSimScale; dR--; delayMicroseconds(5); digitalWriteFast(RStepPin,LOW); }
          }
          if(dA!=0){ Go=true; ASim+=dASim;
            if(ASim>GoSimScale){ digitalWriteFast(AStepPin,HIGH); ASim-=GoSimScale; dA--; delayMicroseconds(5); digitalWriteFast(AStepPin,LOW); }
          }
          if(dZ!=0){ Go=true; ZSim+=dZSim;
            if(ZSim>GoSimScale){ digitalWriteFast(ZStepPin,HIGH); ZSim-=GoSimScale; dZ--; delayMicroseconds(5); digitalWriteFast(ZStepPin,LOW); }
          }
          if(dE!=0){ Go=true; ESim+=dESim;
            if(ESim>GoSimScale){ digitalWriteFast(EStepPin,HIGH); ESim-=GoSimScale; dE--; delayMicroseconds(5); digitalWriteFast(EStepPin,LOW); }
          }
        }
        if(Go){delayMicroseconds(GoT);}
      }
      Tstart=micros();
      dVt=0UL+dVT; GoSimt=0UL; MotionPhase=5;
      //PrintPhaseLog();
      if(DepthLog){
        Serial.print("echo/T: "+String(Tstate)+", MotionT: "+String(MotionT)
          +"\ndSim: "+String(dRSim)+", "+String(dASim)+", "+String(dZSim)+", "+String(dESim)+"}");
      }
    }
};

MotionProfile MotionBuffer[MotionBufferSize];



void QueueMotionProfile(){
  if(!NextPresent){
    UpdateCartToRA(XtargCart,YtargCart);
    RtargStep=CartToR;
    AtargStep=CartToA;
    ZtargStep=CartToZ(ZtargCart);
    EtargStep=CartToE(EtargCart);
    vtarg=FeedRate/60*CartToStepK;
    vstart=0;
  }else{
    RtargStep=RnexttargStep;
    AtargStep=AnexttargStep;
    ZtargStep=CartToZ(ZtargCart);
    EtargStep=CartToE(EtargCart);
    vtarg=vnxttarg;
    vstart=QueueProfileLastVend;
  }
  XnexttargCart=XtargCart; YnexttargCart=YtargCart;
  RnexttargStep=RtargStep; AnexttargStep=AtargStep;
  dr=word(abs(long(RtargStep)-long(RStep))); da=word(abs(long(AtargStep)-long(AStep)));
  dz=word(abs(long(ZtargStep)-long(ZStep))); de=word(min(65536,abs(EtargStep-EStep)));
  if(NextPresent){ dStep=dnextStep; }
  else{ dStep=sqrt(sq(float(dr))+sq(float(da))); }

  byte i=(BufferHead+1==BufferSize)?0:(BufferHead+1);
  if(i!=BufferTail && Buffer[i][0]=='G'
      && (Buffer[i][1]=='1' || Buffer[i][1]=='0')
      && (MotionBufferFilled+1<MotionBufferSize)){
    NextPresent=true;

    float attr=GcodeParser(Buffer[i],'F');
    if(attr!=987654321){ vnxttarg=attr/60*CartToStepK; }else{vnxttarg=vtarg;}
      
    attr=GcodeParser(Buffer[i],'X');
    if(attr!=987654321){ if(AbsolutePositioning){ XnexttargCart=attr; }else{ XnexttargCart+=attr; }}
    attr=GcodeParser(Buffer[i],'Y');
    if(attr!=987654321){ if(AbsolutePositioning){ YnexttargCart=attr; }else{ YnexttargCart+=attr; }}
    
    if(XnexttargCart!=XtargCart||YnexttargCart!=YtargCart){ 
      UpdateCartToRA(XnexttargCart,YnexttargCart);
      RnexttargStep=CartToR; AnexttargStep=CartToA;
      dnextStep=sqrt(sq(float(RnexttargStep)-float(RtargStep))+sq(float(AnexttargStep)-float(AtargStep)));
      if(dStep!=0){
        vend=min(sqrt(2*AccelStep*dnextStep),min(vtarg,vnxttarg*sq(sq(sq(max(0,
          ((float(RtargStep)-float(RStep))*(float(RnexttargStep)-float(RtargStep))+(float(AtargStep)-float(AStep))*(float(AnexttargStep)-float(AtargStep)))/
          (dStep*dnextStep)))))));
      }else{
        vend=0;
      }
    }else{
      vend=0;
      dnextStep=0;
    }
  }else{
    NextPresent=false;
    vnxttarg=vtarg; vend=0;
    dnextStep=0;
  }
  
  QueueProfileLastVend=vend;
  vend=max(MinVend,vend);
  MotionBuffer[MotionBufferFilled].MotionSetup(vtarg,vstart,vend, dr,da,dz,de,(RtargStep>=RStep),(AtargStep>=AStep),(ZtargStep>=ZStep),(EtargStep>=EStep));
  MotionBufferFilled++;

  XCart=XtargCart; YCart=YtargCart;
  ZCart=ZtargCart; ECart=EtargCart;
  RStep=RtargStep; AStep=AtargStep;
  ZStep=ZtargStep; EStep=EtargStep;

  FinishedCode=true;
}

void RunMotionQueue(){
  if(ThermalOn && HotendHeaterOn){
    analogWrite(HotendHeaterPin,byte(HotendPDIdealEffort*255+0.45));
  }
  for(byte i=0; i<MotionBufferFilled; i++){
    MotionBuffer[i].Prepare();
  }
  TurnOnMotors();
  Serial.print("run}");
  Tstart=micros();
  for(byte i=0; i<MotionBufferFilled; i++){
    MotionBuffer[i].GoTarget();
  }
  Serial.print("endrun}");
  TurnOffMotors();
  MotionBufferFilled=0;
}



//----------------------MOTION PROFILE-----------------------------






//-----------------GCODE INTERPRETING AND EXECUTION---------------------

//given a Gcode and a char attribute, return the value or the error code
float GcodeParser(String str, char a){
  String val="";
  int n=str.indexOf(a);
  if(n!=-1){
    while((str[n+1]-'0'>=0 && str[n+1]-'0'<=9) || str[n+1]=='-' || str[n+1]=='.'){
      val+=str[n+1];
      n++;
    }
    return val.toFloat();
  }else{
    return 987654321;
  }
}

//given a Gcode, updates the variables modified by the attributes and calls the comand
void GcodeCall(String str){
  int code=-1;
  if(str[0]=='C'){
    //update the possible attributes of the G codes
    float attr=0;
    attr=GcodeParser(str,'X');
    if(attr!=987654321){ C=attr; }
    
    code=GcodeParser(str,'C');
    switch(code){
      case 0:C0();break;
      case 1:C1();break;
      case 2:C2();break;
      default:unknown(str);break;
    }
  }else if(str[0]=='G'){
    //update the possible attributes of the G codes
    X=GcodeParser(str,'X');
    Y=GcodeParser(str,'Y');
    Z=GcodeParser(str,'Z');
    E=GcodeParser(str,'E');
    F=GcodeParser(str,'F');
    
    code=GcodeParser(str,'G');
    switch(code){
      case 0:
      case 1:G1();break;
      case 28:G28();break;
      case 90:G90();break;
      case 91:G91();break;
      case 92:G92();break;
      default:unknown(str);break;
    }
  }else if(str[0]=='M'){
    //update the possible attributes of the M codes
    S=GcodeParser(str,'S');

    code=GcodeParser(str,'M');
    switch(code){
      case 83:M83();break;
      case 84:M84();break;
      case 104:M104();break;
      case 106:M106();break;
      case 107:M107();break;
      case 109:M109();break;
      case 140:M140();break;
      default:unknown(str);break;
    }
  }else if(str[0]=='T'){

    code=GcodeParser(str,'T');
    switch(code){
      case 0:T0();break;
      default:unknown(str);break;
    }
  }else{
    unknown(str);
  }
}
//if the software doesnt know
void unknown(String str){
  Serial.print("echo/received unknown Gcode: "+String(str)+"}");
  delay(4000);
  FinishedCode=true;
}  

//C codes, customized for mb3d software
void C0(){
  DepthLog=(C==1);
  FinishedCode=true;
}
void C1(){
  FindEndstop(C);
  FinishedCode=true;
}
void C2(){
  DepthLogPlus=(C==1);
  FinishedCode=true;
}


//G codes
void G0(){
  delay(5000);
  FinishedCode=true;
}
void G1(){
  MotionCode=true;
  if(F!=987654321){FeedRate=F;}
  if(X!=987654321){if(AbsolutePositioning){XtargCart=X;}else{XtargCart+=X;}}
  if(Y!=987654321){if(AbsolutePositioning){YtargCart=Y;}else{YtargCart+=Y;}}
  if(Z!=987654321){if(AbsolutePositioning){ZtargCart=Z;}else{ZtargCart+=Z;}}
  if(E!=987654321){if(AbsoluteExtrusion){EtargCart=E;}else{EtargCart+=E;}}
  
  QueueMotionProfile();
}
void G28(){ 
  boolean x=false; boolean y=false; boolean z=false;
  if(X!=987654321){x=true; }
  if(Y!=987654321){y=true; }
  if(Z!=987654321){z=true;}
  if(X==987654321 && Y==987654321 && Z==987654321){ x=true; y=true; z=true; }
  FeedRate=StandardFeedRate;
  if(DepthLog){ Serial.print("echo/started homing: "+String(x)+", "+String(y)+", "+String(z)+"}"); }

  UpdateCartToRA(XHome,YHome);
  int rhome=CartToR; int ahome=CartToA; word zhome=CartToZ(ZHome);

  if(x){FindEndstop(1); digitalWrite(RDirPin,HIGH); RtargStep=rhome; dr=word(abs(long(RtargStep)-long(RStep)));}else{dr=0;}
  Time=millis(); ThermalTimerUpdate();
  if(y){FindEndstop(2); digitalWrite(ADirPin,HIGH); AtargStep=ahome; da=word(abs(long(AtargStep)-long(AStep)));}else{da=0;}
  Time=millis(); ThermalTimerUpdate();
  if(z){FindEndstop(3); digitalWrite(ZDirPin,LOW);  ZtargStep=zhome; dz=word(abs(long(ZtargStep)-long(ZStep)));}else{dz=0;}
  Time=millis(); ThermalTimerUpdate();

  dr*=4; da*=4; //for temporary 2mm lead screw setup----------------------------------

  MotionProfile homing=MotionProfile();  
  homing.MotionSetup(FeedRate/60*CartToStepK,0,0,dr,da,dz,0,(rhome>=RStep),(ahome>=AStep),(zhome>=ZStep),true);
  homing.Prepare();
  TurnOnMotors(); delay(1);
  homing.GoTarget();
  TurnOffMotors();

  if(x){ RStep=rhome;}
  if(y){ AStep=ahome;} 
  if(z){ ZStep=zhome;}
  UpdateStepToXY(RStep,AStep);
  XCart=StepToX; YCart=StepToY;
  ZCart=StepToZ(ZStep);
  XtargCart=XCart; YtargCart=YCart; ZtargCart=ZCart; EtargCart=ECart;

  if(DepthLog){ Serial.print("echo/ended homing, "+String(XCart)+", "+String(YCart)+", "+String(ZCart)+"}"); }
  FinishedCode=true;
}
void G90(){ AbsolutePositioning=true; FinishedCode=true; }
void G91(){ AbsolutePositioning=false; FinishedCode=true; }
void G92(){
  if(X!=987654321){XCart=X;}
  if(Y!=987654321){YCart=Y;}
  if(Z!=987654321){ZCart=Z;}
  if(E!=987654321){ECart=E;}
  UpdateCartToRA(XCart,YCart);
  RStep=CartToR; AStep=CartToA;
  ZStep=CartToZ(ZCart); EStep=CartToE(ECart);

  FinishedCode=true;
}

//M codes
void M82(){ AbsoluteExtrusion=true; FinishedCode=true; }
void M83(){ AbsoluteExtrusion=false; FinishedCode=true; }
void M84(){ FinishedCode=true; }
void M104(){
  if(S!=987654321){ if(!ThermalOn){ToggleThermal();}HotendUpdateTempTarg(); }
  else{ ToggleThermal(); }
  FinishedCode=true;
}
void M106(){
  if(S!=987654321){
    if(S==0){ TogglePrintFan(false); }
    else{ TogglePrintFan(true); }
  }else{
    TogglePrintFan(!PrintFanOn);
  }
  FinishedCode=true;
}
void M107(){
  TogglePrintFan(false);
  FinishedCode=true;
}
void M109(){
  if(!ThermalOn){ ToggleThermal(); }
  if(S!=987654321){ HotendUpdateTempTarg(); }
  HotendTempTargWait();
  FinishedCode=true;
}
void M140(){ FinishedCode=true; }

//T codes
void T0(){ FinishedCode=true; }

//-----------------GCODE INTERPRETING AND EXECUTION--------------------"
