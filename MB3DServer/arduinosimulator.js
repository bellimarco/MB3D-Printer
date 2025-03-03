

module.exports={

  //used outside the module
  MotionToShift:0,
  MotionToPush:[],

  //global
  PivotToTangent:428,
  NozzleToR:25,
  PivotX:279,
  RStepsPerMM:25,
  AStepsPerMM:25,
  ZStepsPerMM:100,
  EStepsPerMM:24.4,
  RMicrostep:4,
  AMicrostep:4,
  ZMicrostep:4,
  EMicrostep:8,
  REndstopPosition:-8750, //steps endstop coordinates
  AEndstopPosition:-11250,
  ZEndstopPosition:32700,
  XHome:0, //cartesian home coordinates
  YHome:0,
  ZHome:75,

  CartToStepK:100,
  MinVend:0,
  CartToExtrK:1,
  VtargToVZDefaultK:0.8,
  StandardFeedRate:1000,
  MaxPredictedVelocity:8000,


  //SYSTEM VARIABLES
  DepthLog:false,
  //gcode buffer
  BufferSize:8,
  Buffer:[],
  BufferIndex:[],//the index of the code in the Buffer
  BufferHead:0,
  BufferTail:0,
  BufferFilled:0,

  MotionBufferSize:120,
  MotionBuffer:[],
  MotionBufferHead:0,
  MotionBufferFilled:0,

  Running:false,

  FinishedCode:false,

  Ron:false, Aon:false,
  Zon:false, Eon:false,
  Ractive:false, Aactive:false,
  Zactive:false, Eactive:false,


  //STATE VARIABLES
  //global variables for queuing and running MotionProfiles
  MotionCode:false,
  NextPresent:false,
  //current coordinates
  XCart:0, YCart:0,
  ZCart:0, ECart:0,
  RStep:0, AStep:0,
  ZStep:0, EStep:0,
  //target coordinates
  XtargCart:0, YtargCart:0,
  ZtargCart:0, EtargCart:0,
  RtargStep:0, AtargStep:0,
  ZtargStep:0, EtargStep:0,
  //other
  XnexttargCart:0, YnexttargCart:0,
  RnexttargStep:0, AnexttargStep:0,
  dStep:0, dnextStep:0,

  //RA motion profile global variables
  AccelCart:180,
  AccelStep:0,
  QueueProfileLastVend:0,

  //other state variables
  FeedRate:1000,
  AbsoluteExtrusion:false,
  AbsolutePositioning:true,

  //Gcode received attributes, modified only directly by Gcodes
  C:0,
  X:0, Y:0, Z:0, E:0,
  F:0,
  S:0,


  //thermal management variables
  ThermalOn:false, //if any thermal related stuff is working
  HotendHeaterOn:false,  HotendFanOn:false, PrintFanOn:false,
  Kheating:2.8991, //calculated heating constant (degrees/s/effort)
  Kcooling:0.004431, //calculated newton cooling constant (1/s)
  KcoolingFan:0.0092, //calculated newton cooling constant with active print fan
  HotendPDKp:0.04, //effort to increase per degree error
  Tamb:20,
  HotendTemp:20,
  HotendTempTarg:20,
  HotendOldTemp:20,
  HotendOld2Temp:20,
  HotendPDIdealEffort:0, //effort to maintain stability at TempTarg
  HotendPDError:0,
  HotendPDEffort:0, //effort



  Setup:function(){
    this.AccelStep=this.CartToStepK*this.AccelCart;
    this.MinVend=this.CartToStepK*1.8;

    for(var i=0; i<this.BufferSize; i++){ this.Buffer[i]="/"; }
    for(var i=0; i<this.MotionBufferSize; i++){
     this.MotionBuffer[i]={
       index:-1,
       XCart:0, YCart:0,
       ZCart:0, ECart:0,
       RStep:0, AStep:0,
       ZStep:0, EStep:0,
       XtargCart:0, YtargCart:0,
       ZtargCart:0, EtargCart:0,
       RtargStep:0, AtargStep:0,
       ZtargStep:0, EtargStep:0,
       Vstart:0, Vtarg:0, Vend:0,
       T:0,
     };
    }
  },

  AddBuffer:function(str,i){
    this.Buffer[this.BufferTail]=str;
    this.BufferIndex[this.BufferTail]=i;
    this.BufferTail=(this.BufferTail+1==this.BufferSize)?0:(this.BufferTail+1);
    this.BufferFilled++;
  },

  //finish the current code
  FinishCode:function(){
    if(this.MotionCode && !this.NextPresent){
      this.RunMotionQueue();
    }

    this.MotionCode=false;
    this.FinishedCode=false;
    if(this.DepthLog){console.log("ArduSim/FinishedCode: "+this.Buffer[this.BufferHead]+", filled: "+(this.BufferFilled-1));}

    this.Buffer[this.BufferHead]="/";
    this.BufferHead=(this.BufferHead+1==this.BufferSize)?0:(this.BufferHead+1);
    this.BufferFilled--;

    if(this.BufferFilled>0){
      this.NextCode();
    }
  },
  //call next code in the buffer
  NextCode:function(){
    if(this.DepthLog){console.log("ArduSim/executing "+this.Buffer[this.BufferHead]);}
    this.GcodeCall(this.Buffer[this.BufferHead]);
  },


  TurnOffMotors:function(){
    if(this.Ractive){ this.Ractive=false; this.Ron=false; }
    if(this.Aactive){ this.Aactive=false; this.Aon=false; }
    if(this.Zactive){ this.Zactive=false; this.Zon=false; }
    if(this.Eactive){ this.Eactive=false; this.Eon=false; }
  },
  TurnOnMotors:function(){
    if(this.Ractive){ this.Ron=true; }
    if(this.Aactive){ this.Aon=true; }
    if(this.Zactive){ this.Zon=true; }
    if(this.Eactive){ this.Eon=true; }
  },

  //conversion
  //cartesian to step conversion
  CartToR:0, CartToA:0,
  UpdateCartToRA:function(x,y){
    var a2=(y+this.NozzleToR)*this.PivotToTangent/(this.PivotX-x);
    var r1=Math.sqrt((this.PivotX-x)*(this.PivotX-x)+y*y+2*y*this.NozzleToR);
    var a1=(a2-this.PivotToTangent*this.NozzleToR/r1)/(1+a2*this.NozzleToR/(this.PivotToTangent*r1));

    this.CartToR=Math.round((this.PivotX-r1)*this.RStepsPerMM*this.RMicrostep);
    this.CartToA=Math.round(a1*this.AStepsPerMM*this.AMicrostep);
  },
  CartToZ:function(z){ return (Math.max(0,z*this.ZStepsPerMM*this.ZMicrostep)); },
  CartToE:function(e){ return e*this.EStepsPerMM*this.EMicrostep; },
  //step to cartesian conversion
  StepToX:0, StepToY:0,
  UpdateStepToXY:function(r,a){
    var r1=(this.PivotX-r/(this.RStepsPerMM*this.RMicrostep)); var a1=a/(this.AStepsPerMM*this.AMicrostep);
    var r2=Math.sqrt(this.NozzleToR*this.NozzleToR+r1*r1); var a2=(a1+this.PivotToTangent*this.NozzleToR/r1)/(1-a1*this.NozzleToR/(this.PivotToTangent*r1));
    var c=Math.sqrt(this.PivotToTangent*this.PivotToTangent+a2*a2);
    this.StepToX=(r2*this.PivotToTangent/c-this.PivotX).toFixed(2);
    this.StepToY=(r2*a2/c-this.NozzleToR).toFixed(2);
  },
  StepToZ:function(z){
    return (z/(this.ZStepsPerMM*this.ZMicrostep)).toFixed(2);
  },
  StepToE:function(e){
    return (e/(this.EStepsPerMM*this.EMicrostep)).toFixed(3);
  },

  FindEndstop:function(n){
    switch (n){
      case 1: this.RStep=this.REndstopPosition; this.RtargStep=this.RStep; break;
      case 2: this.AStep=this.AEndstopPosition; this.AtargStep=this.AStep; break;
      case 3: this.ZStep=this.ZEndstopPosition; this.ZtargStep=this.ZStep; break;
    }
    this.UpdateStepToXY(this.RStep,this.AStep);
    this.XCart=this.StepToX; this.YCart=this.StepToY;
    this.ZCart=this.StepToZ(this.ZStep); this.ECart=this.StepToE(this.EStep);
    this.XtargCart=this.XCart; this.YtargCart=this.YCart; this.ZtargCart=this.ZCart; this.EtargCart=this.ECart;
  },



  //----------------------MOTION PROFILE-----------------------------

  QueueMotionProfile:function(){
    if(this.NextPresent){
      this.RtargStep=this.RnexttargStep;
      this.AtargStep=this.AnexttargStep;
      this.ZtargStep=this.CartToZ(this.ZtargCart);
      this.EtargStep=this.CartToE(this.EtargCart);
      this.vtarg=this.vnxttarg;
      this.vstart=this.QueueProfileLastVend;
    }else{
      this.UpdateCartToRA(this.XtargCart,this.YtargCart);
      this.RtargStep=this.CartToR;
      this.AtargStep=this.CartToA;
      this.ZtargStep=this.CartToZ(this.ZtargCart);
      this.EtargStep=this.CartToE(this.EtargCart);
      this.vtarg=this.FeedRate/60*this.CartToStepK;
      this.vstart=0;
    }
    this.XnexttargCart=this.XtargCart; this.YnexttargCart=this.YtargCart;
    this.RnexttargStep=this.RtargStep; this.AnexttargStep=this.AtargStep;

    this.MotionBuffer[this.MotionBufferFilled].index=this.BufferIndex[this.BufferHead];
    this.MotionBuffer[this.MotionBufferFilled].XCart=this.XCart;
    this.MotionBuffer[this.MotionBufferFilled].YCart=this.YCart;
    this.MotionBuffer[this.MotionBufferFilled].ZCart=this.ZCart;
    this.MotionBuffer[this.MotionBufferFilled].ECart=this.ECart;
    this.MotionBuffer[this.MotionBufferFilled].RStep=this.RStep;
    this.MotionBuffer[this.MotionBufferFilled].AStep=this.AStep;
    this.MotionBuffer[this.MotionBufferFilled].ZStep=this.ZStep;
    this.MotionBuffer[this.MotionBufferFilled].EStep=this.EStep;
    this.MotionBuffer[this.MotionBufferFilled].XtargCart=this.XtargCart;
    this.MotionBuffer[this.MotionBufferFilled].YtargCart=this.YtargCart;
    this.MotionBuffer[this.MotionBufferFilled].ZtargCart=this.ZtargCart;
    this.MotionBuffer[this.MotionBufferFilled].EtargCart=this.EtargCart;
    this.MotionBuffer[this.MotionBufferFilled].RtargStep=this.RtargStep;
    this.MotionBuffer[this.MotionBufferFilled].AtargStep=this.AtargStep;
    this.MotionBuffer[this.MotionBufferFilled].ZtargStep=this.ZtargStep;
    this.MotionBuffer[this.MotionBufferFilled].EtargStep=this.EtargStep;

    var dr=Math.abs(this.RtargStep-this.RStep); var da=Math.abs(this.AtargStep-this.AStep);
    var dz=Math.abs(this.ZtargStep-this.ZStep); var de=Math.abs(this.EtargStep-this.EStep);
    if(this.NextPresent){ this.dStep=this.dnextStep; }
    else{ this.dStep=Math.sqrt(dr*dr+da*da); }
    this.dnextStep=0;

    var i=(this.BufferHead+1==this.BufferSize)?0:(this.BufferHead+1);
    if(i!=this.BufferTail && this.Buffer[i][0]=='G'
        && (this.Buffer[i][1]=='1' || this.Buffer[i][1]=='0')
        && (this.MotionBufferFilled+1<this.MotionBufferSize)){
      this.NextPresent=true;

      var attr=this.GcodeParser(this.Buffer[i],'F');
      if(attr!=987654321){ this.vnxttarg=attr/60*this.CartToStepK; }else{this.vnxttarg=this.vtarg;}

      attr=this.GcodeParser(this.Buffer[i],'X');
      if(attr!=987654321){ if(this.AbsolutePositioning){ this.XnexttargCart=attr; }else{ this.XnexttargCart+=attr; }}
      attr=this.GcodeParser(this.Buffer[i],'Y');
      if(attr!=987654321){ if(this.AbsolutePositioning){ this.YnexttargCart=attr; }else{ this.YnexttargCart+=attr; }}

      if(this.XnexttargCart!=this.XtargCart||this.YnexttargCart!=this.YtargCart){
        if(dr==0 && da==0 && (this.EtargStep-this.EStep)<0){
          vend=0;
          dnextstep=0;
          //if is retracting, end queue without retractat queue
          this.NextPresent=false;
        }
        this.UpdateCartToRA(this.XnexttargCart,this.YnexttargCart);
        this.RnexttargStep=this.CartToR; this.AnexttargStep=this.CartToA;
        this.dnextStep=Math.sqrt((this.RnexttargStep-this.RtargStep)*(this.RnexttargStep-this.RtargStep)+(this.AnexttargStep-this.AtargStep)*(this.AnexttargStep-this.AtargStep));
        if(this.dStep!=0){
          this.vend=Math.min(this.vtarg,this.vnxttarg*Math.pow(Math.max(0,
            ((this.RtargStep-this.RStep)*(this.RnexttargStep-this.RtargStep)+(this.AtargStep-this.AStep)*(this.AnexttargStep-this.AtargStep))/
            (this.dStep*this.dnextStep)),8));
        }else{
          this.vend=0;
        }
      }else{
        this.vend=0;
        this.dnextStep=0;
      }
    }else{
      this.NextPresent=false;
      this.vnxttarg=this.vtarg; this.vend=0;
      this.dnextStep=0;
    }

    this.QueueProfileLastVend=this.vend;
    this.vend=Math.max(this.MinVend,this.vend);
    this.MotionBuffer[this.MotionBufferFilled].Vstart=this.vstart;
    this.MotionBuffer[this.MotionBufferFilled].Vtarg=this.vtarg;
    this.MotionBuffer[this.MotionBufferFilled].Vend=this.vend;

    var phase1_t=0; var phase1_2AS=0;
    var phase2_t=0; var phase2_2AS=0;
    var phase3_t=0; var phase3_2AS=0;
    if(dr!=0 ||da!=0){
      var V1=Math.min(this.vstart,this.vend);
      var V2=Math.max(this.vstart,this.vend);
      phase1_t=(V2-V1)/this.AccelStep;
      phase1_2AS=V2*V2-V1*V1;
      if(this.dStep*2*this.AccelStep<phase1_2AS){
        phase1_2AS=this.dStep*2*this.AccelStep;
        if(this.vend>=this.vstart){
          phase1_t=(Math.sqrt(V1*V1+phase1_2AS)-V1)/this.AccelStep;
        }else{
          phase1_t=(-Math.sqrt(V2*V2-phase1_2AS)+V2)/this.AccelStep;
        }
        this.MotionBuffer[this.MotionBufferFilled].T=phase1_t;
      }else{
        phase2_t=Math.abs(this.vtarg-V2)/this.AccelStep;
        phase2_2AS=2*(this.vtarg*this.vtarg-V2*V2);
        if(this.dStep*2*this.AccelStep-phase1_2AS<phase2_2AS){
          phase2_2AS=this.dStep*2*this.AccelStep-phase1_2AS;
          phase2_t=(Math.sqrt(V2*V2+phase2_2AS/2)-V2)/this.AccelStep;
          this.MotionBuffer[this.MotionBufferFilled].T=phase1_t+2*phase2_t;
        }else{
          phase3_2AS=this.dStep*2*this.AccelStep-phase1_2AS-phase2_2AS;
          phase3_t=phase3_2AS/(this.AccelStep*4*this.vtarg);
          this.MotionBuffer[this.MotionBufferFilled].T=phase1_t+2*phase2_t+2*phase3_t;
        }
      }
    }else{
      if(dz!=0){ this.MotionBuffer[this.MotionBufferFilled].T=dz/(this.vtarg*this.VtargToVZDefaultK); }
      else if(de!=0){ this.MotionBuffer[this.MotionBufferFilled].T=de/(this.vtarg*this.CartToExtrK); }
      else{ this.MotionBuffer[this.MotionBufferFilled].T=0; }
    }

    var topush={ index:-1, dx:0, dy:0, dz:0, de:0, T:0};
    topush.index=this.MotionBuffer[this.MotionBufferFilled].index;
    topush.dx=this.XtargCart-this.XCart;
    topush.dy=this.YtargCart-this.YCart;
    topush.dz=this.ZtargCart-this.ZCart;
    topush.de=this.EtargCart-this.ECart;
    topush.T=this.MotionBuffer[this.MotionBufferFilled].T;
    this.MotionToPush.push(topush);
    this.MotionToPushOn=true;

    this.MotionBufferFilled++;

    this.XCart=this.XtargCart; this.YCart=this.YtargCart;
    this.ZCart=this.ZtargCart; this.ECart=this.EtargCart;
    this.RStep=this.RtargStep; this.AStep=this.AtargStep;
    this.ZStep=this.ZtargStep; this.EStep=this.EtargStep;

    if(this.DepthLog){console.log("ArduSim/Queued motion profile, filled: "+this.MotionBufferFilled);}
    this.FinishedCode=true;
  },

  //shift the motion queue
  AdvanceMotionQueue:function(){
    this.MotionToShiftOn=true;
    this.MotionToShift++;

    console.log("ArduSim/motion completed:  "+this.MotionBufferHead);
    this.MotionBufferHead++;
  },
  //actually this is called only when arduino finishes the queue
  RunMotionQueue:function(){
    if(this.DepthLog){console.log("ArduSim/Ended motion queue");}
  },
  //what actually gets called when runend code recieved from arduino
  MotionQueueEnd:function(){
    this.Running=false;
    while(this.MotionBufferHead<this.MotionBufferFilled){
      this.AdvanceMotionQueue();
    }
    this.MotionBufferFilled=0;
    this.MotionBufferHead=0;
  },





  //-----------------THERMAL MANAGEMENT-------------------------------

  //update temperature and effort
  //data is anyway sent automatically by arduino, no need to periodically request
  RequestThermalUpdate:function(ser){
    ser.write("{therm}")
  },
  ResponseThermalUpdate:function(data){
    var comma=data.indexOf(",");
    this.HotendTemp=parseInt(data.slice(0,comma));
    this.HotendPDEffort=parseFloat(data.slice(comma+1,data.length))/1000;
  },
  ThermalUpdate:function(){
  },
  //update target temp and ideal effort
  HotendUpdateTempTarg:function(){
    this.HotendTempTarg=Math.round(this.S);
    this.HotendPDIdealEffort=Math.round((this.HotendTempTarg-this.Tamb)*(this.PrintFanOn?this.KcoolingFan:this.Kcooling)/this.Kheating);
  },

  //tharmal controls
  ToggleHotendFan:function(n){
    if(!n){this.HotendFanOn=false;}
    else{this.HotendFanOn=true;}
  },
  ToggleHotendHeater:function(n){
    if(!n){this.HotendHeaterOn=false;}
    else{this.HotendHeaterOn=true;}
  },
  TogglePrintFan:function(n){
    if(!n){this.PrintFanOn=false;}
    else{this.PrintFanOn=true;}
  },
  ToggleThermal:function(){
    this.ThermalOn=!this.ThermalOn;
    this.HotendTempTarg=20; this.HotendPDIdealEffort=0;
    this.HotendOld2Temp=20; this.HotendOldTemp=20;
    this.HotendPDEffort=0;
    this.ThermalUpdate();
    if(this.ThermalOn){ this.ToggleHotendHeater(true); this.ToggleHotendFan(true); }
    else{ this.ToggleHotendHeater(false); this.ToggleHotendFan(false); this.TogglePrintFan(false);}
  },




  //-----------------GCODE INTERPRETING AND EXECUTION---------------------

  //given a Gcode and a char attribute, return the value or the error code
  GcodeParser:function(str,a){
    var val="";
    var n=str.indexOf(a);
    if(n!=-1){
      val="";
      while(str.charAt(n+1).length==1 && ((str.charAt(n+1)-'0'>=0 && str.charAt(n+1)-'0'<=9) || str.charAt(n+1)=='-' || str.charAt(n+1)=='.')){
        val+=str.charAt(n+1);
        n++;
      }
      if(val.length==0){val="0"};
      return parseFloat(val);
    }else{
      return 987654321;
    }
  },
  //given a Gcode, updates the variables modified by the attributes and calls the comand
  GcodeCall:function(str){
    var code=-1;
    if(str.charAt(0)=='C'){
      var attr=this.GcodeParser(str,'X');
      if(attr!=987654321){ this.C=attr; }

      code=this.GcodeParser(str,'C');
      switch(code){
        case 0:this.C0();break;
        case 1:this.C1();break;
        case 2:this.C2();break;
        case 3:this.C3();break;
        default:this.unknown(str);break;
      }
    }else if(str.charAt(0)=='G'){
      this.X=this.GcodeParser(str,'X');
      this.Y=this.GcodeParser(str,'Y');
      this.Z=this.GcodeParser(str,'Z');
      this.E=this.GcodeParser(str,'E');
      this.F=this.GcodeParser(str,'F');

      code=this.GcodeParser(str,'G');
      switch(code){
        case 0:
        case 1:this.G1();break;
        case 28:this.G28();break;
        case 90:this.G90();break;
        case 91:this.G91();break;
        case 92:this.G92();break;
        default:this.unknown(str);break;
      }
    }else if(str.charAt(0)=='M'){
      this.S=this.GcodeParser(str,'S');

      code=this.GcodeParser(str,'M');
      switch(code){
        case 83:this.M83();break;
        case 84:this.M84();break;
        case 104:this.M104();break;
        case 106:this.M106();break;
        case 107:this.M107();break;
        case 109:this.M109();break;
        case 140:this.M140();break;
        default:this.unknown(str);break;
      }
    }else if(str.charAt(0)=='T'){

      code=this.GcodeParser(str,'T');
      switch(code){
        case 0:this.T0();break;
        default:this.unknown(str);break;
      }
    }else{
      this.unknown(str);
    }
  },

  unknown:function(str){this.FinishedCode=true;},
  //C codes, customized for mb3d software
  C0:function(){this.FinishedCode=true;},
  C1:function(){
    this.FindEndstop(this.C);
    this.FinishedCode=true;
  },
  C2:function(){this.FinishedCode=true;},
  C3:function(){this.FinishedCode=true;},
  //G codes
  G0:function(){this.G1();},
  G1:function(){
    this.MotionCode=true;
    if(this.F!=987654321){this.FeedRate=this.F;}
    if(this.X!=987654321){if(this.AbsolutePositioning){this.XtargCart=this.X;}else{this.XtargCart+=this.X;}}
    if(this.Y!=987654321){if(this.AbsolutePositioning){this.YtargCart=this.Y;}else{this.YtargCart+=this.Y;}}
    if(this.Z!=987654321){if(this.AbsolutePositioning){this.ZtargCart=this.Z;}else{this.ZtargCart+=this.Z;}}
    if(this.E!=987654321){if(this.AbsoluteExtrusion){this.EtargCart=this.E;}else{this.EtargCart+=this.E;}}

    this.QueueMotionProfile();
  },
  G28:function(){
    var x=false; var y=false; var z=false;
    if(this.X!=987654321){x=true; }
    if(this.Y!=987654321){y=true; }
    if(this.Z!=987654321){z=true;}
    if(this.X==987654321 && this.Y==987654321 && this.Z==987654321){ x=true; y=true; z=true; }

    this.UpdateCartToRA(this.XHome,this.YHome);
    var rhome=this.CartToR; var ahome=this.CartToA; var zhome=this.CartToZ(this.ZHome);

    if(x){this.FindEndstop(1); this.RtargStep=rhome; this.RStep=rhome;}
    if(y){this.FindEndstop(2); this.AtargStep=ahome; this.AStep=ahome;}
    if(z){this.FindEndstop(3); this.ZtargStep=zhome; this.ZStep=zhome;}

    this.UpdateStepToXY(this.RStep,this.AStep);
    this.XCart=this.StepToX; this.YCart=this.StepToY;
    this.ZCart=this.StepToZ(this.ZStep);
    this.XtargCart=this.XCart; this.YtargCart=this.YCart; this.ZtargCart=this.ZCart;

    this.FinishedCode=true;
  },
  G90:function(){ this.AbsolutePositioning=true; this.FinishedCode=true; },
  G91:function(){ this.AbsolutePositioning=false; this.FinishedCode=true; },
  G92:function(){
    if(this.X!=987654321){this.XCart=this.X;}
    if(this.Y!=987654321){this.YCart=this.Y;}
    if(this.Z!=987654321){this.ZCart=this.Z;}
    if(this.E!=987654321){this.ECart=this.E;}
    this.UpdateCartToRA(this.XCart,this.YCart);
    this.RStep=this.CartToR; this.AStep=this.CartToA;
    this.ZStep=this.CartToZ(this.ZCart); this.EStep=this.CartToE(this.ECart);

    this.FinishedCode=true;
  },
  //M codes
  M82:function(){ this.AbsoluteExtrusion=true; this.FinishedCode=true; },
  M83:function(){ this.AbsoluteExtrusion=false; this.FinishedCode=true; },
  M84:function(){ this.FinishedCode=true; },
  M104:function(){
    if(this.S!=987654321){ if(!this.ThermalOn){this.ToggleThermal();} this.HotendUpdateTempTarg(); }
    else{ this.ToggleThermal(); }
    this.FinishedCode=true;
  },
  M106:function(){
    if(this.S!=987654321){
      if(this.S==0){ this.TogglePrintFan(false); }
      else{ this.TogglePrintFan(true); }
    }else{
      this.TogglePrintFan(!this.PrintFanOn);
    }
    this.FinishedCode=true;
  },
  M107:function(){
    this.TogglePrintFan(false);
    this.FinishedCode=true;
  },
  M109:function(){
    if(!this.ThermalOn){ this.ToggleThermal(); }
    if(this.S!=987654321){ this.HotendUpdateTempTarg(); }
    this.FinishedCode=true;
  },
  M140:function(){ this.FinishedCode=true; },
  //T codes
  T0:function(){ this.FinishedCode=true; },


}
