

function httpGet(url, callback){
  var xmlHttp = new XMLHttpRequest();
  xmlHttp.onreadystatechange = function() {
      if (xmlHttp.readyState == 4 && xmlHttp.status == 200)
          callback(xmlHttp.responseText);
  }
  xmlHttp.open("GET", url, true);
  xmlHttp.send(null);
}


$(document).ready(()=>{

  UpdateSettings();
  UpdateState();
  UpdateBuffers();
});


//Printer settings
var NozzleToR=26;
var PivotToTangent=392;
var PivotX=243;
var RStepsPerMM=25; var RMicrostep=4;
var AStepsPerMM=25; var AMicrostep=4;
var ZStepsPerMM=100; var ZMicrostep=4;
var EStepsPerMM=24.44; var EMicrostep=8;

//System variables
var IsPrinting=false;
var FileSize=0; var GcodesRemaining=0; var CurrentIndex=0;
var GcodeBuffer=[]; var MotionBuffer=[];
var UpdateLoopActive=false; //periodically update

//Motion variables, influenced by the completed motion codes
var XCart=0, YCart=0; ZCart=0; ECart=0;
var RStep=0; AStep=0; ZStep=0; EStep=0;

//other state variables
var PrintFan=0;
var HotFan=0;
var HotTemp=0;
var HotTarg=0;
var HotEffort=0;

//cartesian to step conversion
function UpdateCartToSteps(){
  var a2=(YCart+NozzleToR)*PivotToTangent/(PivotX-XCart); var r2=Math.sqrt((PivotX-XCart)*(PivotX-XCart)+(YCart+NozzleToR)*(YCart+NozzleToR));
  var r1=Math.sqrt(r2*r2-NozzleToR*NozzleToR);
  var a1=(a2-PivotToTangent*NozzleToR/r1)/(1+a2*NozzleToR/(PivotToTangent*r1));
  RStep=Math.round((PivotX-r1)*RStepsPerMM*RMicrostep);
  AStep=Math.round(a1*AStepsPerMM*AMicrostep);

  ZStep=Math.round(Math.max(0,ZCart*ZStepsPerMM*ZMicrostep));
  EStep=Math.round(ECart*EStepsPerMM*EMicrostep);
}



function UpdateLoop(){
  httpGet("/UpdateLoop",(res)=>{
    var a=JSON.parse(res);
    if(a.on){
      for(var i=0; i<a.GPush.length; i++){
        GcodeBufferPush(a.GPush[i]);
      }
      for(var i=0; i<a.MPush.length; i++){
        MotionBufferPush(a.MPush[i]);
      }
      for(var i=0; i<a.GShift; i++){
        GcodeBufferShift();
      }
      if(a.MShift>0){
        FastUpdateState();
        for(var i=0; i<a.MShift; i++){
          MotionBufferShift();
        }
      }
      if(a.updt){
        UpdateState();
      }
    }
  });
}
var UpdateLoopTslow=4000;
var UpdateLoopTfast=300;
var UpdateLoopID=setInterval(UpdateLoop,UpdateLoopTslow);

var ThermUpdateLoopOn=false;
var ThermUpdateLoopT=1000;
var ThermUpdateLoopID=-1;
function ToggleThermUpdateLoop(){
  if(ThermUpdateLoopOn){
    ThermUpdateLoopOn=false;
    clearInterval(ThermUpdateLoopID);
    $("#ToggleThermUpdateLoop").css("background-color","inherit");
  }else{
    ThermUpdateLoopOn=true;
    ThermUpdateLoopID=setInterval(FastUpdateTherm,ThermUpdateLoopT);
    $("#ToggleThermUpdateLoop").css("background-color","lawngreen");
  }
}
$("#ToggleThermUpdateLoop").click(ToggleThermUpdateLoop);


//LIVE GUI----------------------------------------------------------

function UpdateSettings(){
  httpGet("/UpdateSettings",(res)=>{
    var a=JSON.parse(res);
    NozzleToR=a.NozzleToR;
    PivotToTangent=a.PivotToTangent;
    PivotX=a.PivotX;
    RStepsPerMM=a.RStepsPerMM; RMicrostep=a.RMicrostep;
    AStepsPerMM=a.AStepsPerMM;  AMicrostep=a.AMicrostep;
    ZStepsPerMM=a.ZStepsPerMM; ZMicrostep=a.ZMicrostep;
    EStepsPerMM=a.EStepsPerMM; EMicrostep=a.EMicrostep;
  });
}
//Update every state variable
function UpdateState(){
  httpGet("/UpdateState",(res)=>{
    var a=JSON.parse(res);
    XCart=a.X; YCart=a.Y; ZCart=a.Z; ECart=a.E;
    UpdateCartToSteps();
    PrintFan=a.PrintFan;
    HotFan=a.HotFan;
    HotTemp=a.HotTemp;
    HotTarg=a.HotTarg;
    HotEffort=a.HotEffort;
    $("#XCart").text(XCart.toFixed(1));
    $("#YCart").text(YCart.toFixed(1));
    $("#ZCart").text(ZCart.toFixed(1));
    $("#ECart").text(ECart.toFixed(3));
    $("#RStep").text(RStep);
    $("#AStep").text(AStep);
    $("#ZStep").text(ZStep);
    $("#EStep").text(EStep);
    $("#PrintFan").text(PrintFan);
    $("#HotFan").text(HotFan);
    $("#HotTemp").text(HotTemp);
    $("#HotTarg").text(HotTarg);
    $("#HotEffort").text(HotEffort);

    if(IsPrinting!=a.IsPrinting){
      IsPrinting=a.IsPrinting;
      if(IsPrinting){
        clearInterval(UpdateLoopID);
        UpdateLoopID=setInterval(UpdateLoop,UpdateLoopTfast);
      }else{
        clearInterval(UpdateLoopID);
        UpdateLoopID=setInterval(UpdateLoop,UpdateLoopTslow);
      }
    }
  });
}
//update just current coordinates
function FastUpdateState(){
  httpGet("/FastUpdateState",(res)=>{
    var a=JSON.parse(res);
    XCart=a.X; YCart=a.Y; ZCart=a.Z; ECart=a.E;
    UpdateCartToSteps();
    $("#XCart").text(XCart.toFixed(1));
    $("#YCart").text(YCart.toFixed(1));
    $("#ZCart").text(ZCart.toFixed(1));
    $("#ECart").text(ECart.toFixed(3));
    $("#RStep").text(RStep);
    $("#AStep").text(AStep);
    $("#ZStep").text(ZStep);
    $("#EStep").text(EStep);
    HotTemp=a.HotTemp;
    HotEffort=a.HotEffort;
    $("#HotTemp").text(HotTemp);
    $("#HotEffort").text(HotEffort);
  });
}
//update just current therm variables
function FastUpdateTherm(){
  httpGet("/FastUpdateTherm",(res)=>{
    var a=JSON.parse(res);
    HotTemp=a.HotTemp;
    HotEffort=a.HotEffort;
    $("#HotTemp").text(HotTemp);
    $("#HotEffort").text(HotEffort);
  });
}

$("#Xtraslate").on("keypress",(e)=>{if(e.which==13){SendTraslate();}});
$("#Ytraslate").on("keypress",(e)=>{if(e.which==13){SendTraslate();}});
$("#Ztraslate").on("keypress",(e)=>{if(e.which==13){SendTraslate();}});
$("#Etraslate").on("keypress",(e)=>{if(e.which==13){SendTraslate();}});
function SendTraslate(){
  httpGet("/traslate"
    +"/"+(($("#Xtraslate").val().length==0)?0:$("#Xtraslate").val())
    +"/"+(($("#Ytraslate").val().length==0)?0:$("#Ytraslate").val())
    +"/"+(($("#Ztraslate").val().length==0)?0:$("#Ztraslate").val())
    +"/"+(($("#Etraslate").val().length==0)?0:$("#Etraslate").val()),console.log);
  $("#Xtraslate").val("");
  $("#Ytraslate").val("");
  $("#Ztraslate").val("");
  $("#Etraslate").val("");
}

$("#Xhome").click(()=>{httpGet("/homing/1",console.log);});
$("#Yhome").click(()=>{httpGet("/homing/2",console.log);});
$("#Zhome").click(()=>{httpGet("/homing/3",console.log);});
$("#XYZhome").click(()=>{httpGet("/homing/4",console.log);});

$("#Xsetpos").on("keypress",(e)=>{if(e.which==13){SetPos();}});
$("#Ysetpos").on("keypress",(e)=>{if(e.which==13){SetPos();}});
$("#Zsetpos").on("keypress",(e)=>{if(e.which==13){SetPos();}});
$("#Esetpos").on("keypress",(e)=>{if(e.which==13){SetPos();}});
function SetPos(){
  httpGet("/setpos/"
  +(($("#Xsetpos").val().length>0)?$("#Xsetpos").val():"curr")+"/"
  +(($("#Ysetpos").val().length>0)?$("#Ysetpos").val():"curr")+"/"
  +(($("#Zsetpos").val().length>0)?$("#Zsetpos").val():"curr")+"/"
  +(($("#Esetpos").val().length>0)?$("#Esetpos").val():"curr"),console.log);
  $("#Xsetpos").val("");
  $("#Ysetpos").val("");
  $("#Zsetpos").val("");
  $("#Esetpos").val("");
}


$("#Play").click(()=>{
  httpGet("/TogglePrinting",(res)=>{
    IsPrinting=(parseInt(res)==1)?true:false;
    if(IsPrinting){
      clearInterval(UpdateLoopID);
      UpdateLoopID=setInterval(UpdateLoop,UpdateLoopTfast);
    }else{
      clearInterval(UpdateLoopID);
      UpdateLoopID=setInterval(UpdateLoop,UpdateLoopTslow);
    }
  });
});
$("#SingleCode").click(()=>{
  httpGet("/SendSingleCode",console.log);
});









//Gcode MANAGER----------------------------------------------------------

//request complete list of elements in the buffers
function UpdateBuffers(){
  httpGet("/UpdateBuffers",(res)=>{
    var data=JSON.parse(res);
    GcodeBuffer=data.gcode;
    MotionBuffer=data.motion;
    UpdateGcodeManager();
  });
}
//index in the FileGcode of the last gcode in the buffer
function LastBufferIndex(){
  var last=-1;
  for(let i=0; i<GcodeBuffer.length; i++){
    if(GcodeBuffer[i].index!=-1){
      last=GcodeBuffer[i].index;
    }
  }
}
//index in the FileGcode of the first gcode in the buffer
function FirstBufferIndex(){
  var first=-1;
  for(let i=0; i<GcodeBuffer.length; i++){
    if(GcodeBuffer[i].index!=-1){
      first= GcodeBuffer[i].index;
      break;
    }
  }
  return first;
}

$("#ManUpdateBuffers").click(()=>{
  UpdateBuffers();
});
//update the dom elements in the Gcodemanager
function UpdateGcodeManager(){
  $("#GcodeBuffer").empty(); $("#MotionBuffer").empty();
  $("#GcodeBufferSize").text("Gcode buffer: "+GcodeBuffer.length);
  for(var i=0; i<GcodeBuffer.length; i++){
    var txt=GcodeBuffer[i].index+")  "+GcodeBuffer[i].code;
    var element = '<div id="gcode'+GcodeBuffer[i].index+'" class="BufferLine">'+txt+'</div>';
    $("#GcodeBuffer").append(element);
  }
  $("#MotionBufferSize").text("motion buffer: "+MotionBuffer.length);
  for(var i=0; i<MotionBuffer.length; i++){
    var txt1=""; var txt2=""; var txt3=""; var txt4="";
    var a=MotionBuffer[i].dx;
    if(a!=0){txt1+="X"+((a>0)?" ":"-")+Math.abs(a.toFixed(1))}
    a=MotionBuffer[i].dy;
    if(a!=0){txt2+="Y"+((a>0)?" ":"-")+Math.abs(a.toFixed(1))}
    a=MotionBuffer[i].dz;
    if(a!=0){txt3+="Z"+((a>0)?" ":"-")+Math.abs(a.toFixed(1))}
    a=MotionBuffer[i].de;
    if(a!=0){txt4+="E"+((a>0)?" ":"-")+Math.abs(a.toFixed(1))}
    var txt=txt1;
    if(txt.length!=0){ txt+=",   "; } txt+=txt2;
    if(txt.length!=0){ txt+=",   "; } txt+=txt3;
    if(txt.length!=0){ txt+=",   "; } txt+=txt4;
    var element = '<div id="mbuff'+MotionBuffer[i].index+'" class="BufferLine">'+txt+'</div>';
    $("#MotionBuffer").append(element);
  }

  var n=FirstBufferIndex();
  if(n!=-1){ CurrentIndex=n; GcodesRemaining=FileSize-CurrentIndex-1;}
  $("#CurrentIndex").text("current: "+CurrentIndex);
  $("#GcodesRemaining").text("remaining: "+GcodesRemaining);
}
//functions on single elements of the buffers
function GcodeBufferShift(){
  GcodeBuffer.shift();
  $("#GcodeBuffer .BufferLine").first().remove();
  var n=FirstBufferIndex();

  $("#GcodeBufferSize").text("Gcode buffer: "+GcodeBuffer.length);
  if(n!=-1){ CurrentIndex=n; GcodesRemaining=FileSize-CurrentIndex-1;}
  $("#CurrentIndex").text("current: "+CurrentIndex);
  $("#GcodesRemaining").text("remaining: "+GcodesRemaining);
}
function MotionBufferShift(){
  MotionBuffer.shift();
  $("#MotionBuffer .BufferLine").first().remove();

  $("#MotionBufferSize").text("motion buffer: "+MotionBuffer.length);
}
function GcodeBufferPush(gcode){
  GcodeBuffer.push(gcode);
  $("#GcodeBufferSize").text("Gcode buffer: "+GcodeBuffer.length);
  var txt=GcodeBuffer[GcodeBuffer.length-1].index+")  "+GcodeBuffer[GcodeBuffer.length-1].code;
  var element = '<div id="gcode'+GcodeBuffer[GcodeBuffer.length-1].index+'" class="BufferLine">'+txt+'</div>';
  $("#GcodeBuffer").append(element);
}
function MotionBufferPush(mot){
  MotionBuffer.push(mot)
  $("#MotionBufferSize").text("motion buffer: "+MotionBuffer.length);
  var txt1=""; var txt2=""; var txt3=""; var txt4="";
  var a=MotionBuffer[MotionBuffer.length-1].dx;
  if(a!=0){txt1="X"+((a>0)?" ":"-")+Math.abs(a.toFixed(1))}
  a=MotionBuffer[MotionBuffer.length-1].dy;
  if(a!=0){txt2="Y"+((a>0)?" ":"-")+Math.abs(a.toFixed(1))}
  a=MotionBuffer[MotionBuffer.length-1].dz;
  if(a!=0){txt3="Z"+((a>0)?" ":"-")+Math.abs(a.toFixed(1))}
  a=MotionBuffer[MotionBuffer.length-1].de;
  if(a!=0){txt4="E"+((a>0)?" ":"-")+Math.abs(a.toFixed(1))}
  var txt=txt1;
  if(txt.length!=0 && txt2.length!=0){ txt+=",   "; } txt+=txt2;
  if(txt.length!=0 && txt3.length!=0){ txt+=",   "; } txt+=txt3;
  if(txt.length!=0 && txt4.length!=0){ txt+=",   "; } txt+=txt4;
  var element = '<div id="mbuff'+MotionBuffer[MotionBuffer.length-1].index+'" class="BufferLine">'+txt+'</div>';
  $("#MotionBuffer").append(element);
}

$("#CustomGcode").on("keypress",(e)=>{
  if(e.which==13){
    if($("#CustomGcode").val().length>2){
      httpGet("/AddGcode/"+$("#CustomGcode").val(),console.log);
    }
  }
});









//other----------------------------------------------------------

var lastLoaded="index";
$("#LoadFile").on("keypress",(e)=>{
  if(e.which==13 && $("#LoadFile").val().length!=0){
    httpGet("/LoadGcode/"+$("#LoadFile").val(),(res)=>{
      FileSize=parseInt(res);
      GcodesRemaining=FileSize;
      CurrentIndex=0;
      UpdateGcodeManager();
      $("#FileSize").text("total: "+FileSize);
      console.log("loaded file");
    });
    lastLoaded=$("#LoadFile").val();
  }
});

$("#SendSerial").on("keypress",(e)=>{
  if(e.which==13){
    httpGet("/WriteSerial/"+$("#test").val(),console.log);
  }
});


$("#SetLastIndex").on("keypress",(e)=>{
  if(e.which==13){
    httpGet("/SetLastIndex/"+$("#SetLastIndex").val(),console.log);
  }
});

$("#Startup").click(()=>{
  httpGet("/Startup",console.log)
})

$("#LastZ").click(()=>{
  httpGet("/LastZ",alert)
})
