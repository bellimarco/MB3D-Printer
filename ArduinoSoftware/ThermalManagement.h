//timer
const int ThermalTimerT=2000; unsigned long ThermalTimert=0; //update timer for general thermal stuff (millis)

//Thermal management variables
boolean LogThermal=true;
int HotendThermistorRead=988; //V2*1023/5
float HotendThermistorR2=137.0; //Tamb=20 values
boolean ThermalOn=false; //if any thermal related stuff is working
boolean HotendHeaterOn=false; boolean HotendFanOn=false; boolean PrintFanOn=false;
const float Kheating=2.8991; //calculated heating constant (degrees/s/effort)
const float Kcooling=0.004431; //calculated newton cooling constant (1/s)
const float KcoolingFan=0.009014; //calculated newton cooling constant with print fan (1/s)
const float HotendPDKp=0.04; //effort to increase per degree error
const int Tamb=20;
int HotendTemp=20;
int HotendTempTarg=20;
int HotendOldTemp=20;
int HotendOld2Temp=20;
float HotendPDIdealEffort=0; //effort to maintain stability at TempTarg
int HotendPDError=0;
float HotendPDEffort=0; //effort 0-1

//update temperature and PD variables
void ThermalUpdate(){
  HotendThermistorRead=analogRead(HotendThermistorPin);
  HotendThermistorR2=4.7*HotendThermistorRead/(1023.0-HotendThermistorRead);
  int i=-1; while(i<222){ i++; if(HotendThermistorData[i]<HotendThermistorR2){ break; } }
  HotendTemp=i;
  HotendOld2Temp=HotendOldTemp;
  HotendOldTemp=HotendTemp;
  HotendPDError=HotendTempTarg-HotendTemp; //positive error requires positive effort
  HotendPDEffort=max(0,min(0.9,HotendPDIdealEffort+HotendPDKp*HotendPDError-(HotendTemp-HotendOld2Temp)/(2*ThermalTimerT*Kheating)*1000));
  if(HotendHeaterOn){ analogWrite(HotendHeaterPin, byte(HotendPDEffort*255+0.45)); }
  if(LogThermal){
    Serial.print("therm"+String(HotendTemp)+","+String(byte(HotendPDEffort*100))+"}");
  }
  if(DepthLog){
    Serial.print("echo/thermal update: ");
    Serial.print("T: "+String(HotendTemp)+", Terr: "+String(HotendPDError)+", Eff: "+String(HotendPDEffort)+" = "+String(HotendPDIdealEffort)+" + "+String(HotendPDKp*HotendPDError)+" - "+String((HotendTemp-HotendOld2Temp)/(2*ThermalTimerT*Kheating)*1000)+"}");
  }
}
//update target temp and ideal effort
void HotendUpdateTempTarg(){
  HotendTempTarg=int(S);
  HotendPDIdealEffort=(HotendTempTarg-Tamb)*((PrintFanOn)?KcoolingFan:Kcooling)/Kheating;
  if(DepthLog){ Serial.print("echo/set hotend target temp: "+String(HotendTempTarg)+", IdealEffort: "+String(HotendPDIdealEffort)+"}"); }
}

//tharmal controls
void ToggleHotendFan(boolean n){
  if(n){
    HotendFanOn=true;
    digitalWrite(HotendFanPin,HIGH);
  }else{
    HotendFanOn=false;
    digitalWrite(HotendFanPin,LOW);
  }
  if(DepthLog){ Serial.print("echo/toggled hotend fan: "+String(HotendFanOn)+"}"); }
}
void ToggleHotendHeater(boolean n){
  if(n){
    HotendHeaterOn=true;
    analogWrite(HotendHeaterPin, byte(HotendPDEffort*255+0.45));
  }else{
    HotendHeaterOn=false;
    digitalWrite(HotendHeaterPin, LOW);
  }
  if(DepthLog){ Serial.print("echo/toggled heater: "+String(HotendHeaterOn)+"}"); }
}
void TogglePrintFan(boolean n){
  if(n){
    PrintFanOn=true;
    digitalWrite(PrintFanPin,HIGH);
  }else{
    PrintFanOn=false;
    digitalWrite(PrintFanPin,LOW);
  }
  if(DepthLog){ Serial.print("echo/toggled print fan: "+String(PrintFanOn)+"}"); }
  S=HotendTempTarg;
  HotendUpdateTempTarg();
}

void ToggleThermal(){
  ThermalOn=!ThermalOn;
  HotendThermistorRead=988; HotendThermistorR2=137.0;
  ThermalTimert=0;
  HotendTempTarg=20; HotendPDIdealEffort=0; 
  HotendOld2Temp=20; HotendOldTemp=20;
  HotendPDEffort=0;
  ThermalUpdate();
  Serial.print("echo/toggled thermal: "+String(ThermalOn)+"}");
  if(ThermalOn){ ToggleHotendHeater(true); ToggleHotendFan(true); }
  else{ ToggleHotendHeater(false); ToggleHotendFan(false); TogglePrintFan(false); }
}

//thermal timer update
void ThermalTimerUpdate(){
  if(ThermalOn && ThermalTimert<Time){
    ThermalTimert=Time+ThermalTimerT;
    ThermalUpdate();
  }
}

void HotendTempTargWait(){
  Serial.print("echo/hotend started temptarg wait}");
  while(HotendTemp!=HotendTempTarg){
    Time=millis();
    ThermalTimerUpdate();
    delay(ThermalTimerT);
  }
}
