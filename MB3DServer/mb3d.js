console.clear();
console.log("");
console.log("");
console.log("----------MB3D Starting----------");



const fs = require("fs");

//SERVER AND CLIENT SETUP-------------------------------------------------
const http = require("http");
const express = require("express");
const bodyParser = require("body-parser");

var app = express();
var port= "192.168.1.23";
var portNum=8088;
app.listen(portNum, port, () => {
  console.log("srvr/listening on port: "+port+":"+portNum);

});
app.use(express.static("./Website"));
app.get('/', (req, res) => {
  res.render(__dirname + '/Website/index.html');
});
app.use(bodyParser.urlencoded({
  extended: true
}));
app.use(bodyParser.json());



//ARDUINO SIMULATOR
const AS=require("./arduinosimulator.js");
AS.Setup();


//ARDUINO SERIAL COMMUNICATION--------------------------------------------
const PortName="/dev/cu.usbserial-1440";
const serialport = require('serialport');
const Readline = require('@serialport/parser-readline');
const Serial = new serialport(PortName, { baudRate: 200000 });
const Parser=Serial.pipe(new Readline({delimiter:"}"}));

Serial.on("open",()=>{
  console.log("Serial opened");
});
Parser.on("data",(data)=>{
  if(data=="f"){ FinishedCode(); }
  else if(data=="b"){ FloatingBuffer--; if(IsPrinting){ReplenishBuffer();} }
  else if(data.slice(0,4)=="echo"){ console.log();console.log(data); }
  else if(data=="autonext"){ AS.NextCode(); }
  else if(data.slice(0,5)=="therm"){ AS.ResponseThermalUpdate(data.slice(5,data.length)); }
  else if(data.slice(0,3)=="run"){
    AS.Running=true;
    console.log("RunMotionQueue, filled: "+AS.MotionBufferFilled+"-----------");
    function SimAdvance(){
      //console.log("ArduSim/set function SimAdvance, t: "+AS.MotionBuffer[AS.MotionBufferHead].T);
      setTimeout(()=>{
        if(AS.Running){
          AS.AdvanceMotionQueue();
          if(AS.MotionBufferHead<AS.MotionBufferFilled){ SimAdvance(); }
        }
      },Math.ceil(AS.MotionBuffer[AS.MotionBufferHead].T*1000));
    }
    SimAdvance();
  }
  else if(data=="endrun"){
    console.log("Ended Motion Queue------------");
    AS.MotionQueueEnd();
  }
});
app.get("/WriteSerial/:msg",(req,res)=>{
  Serial.write(req.params.msg);
  console.log("Serial write: "+req.params.msg);
  res.send("sent serial");
});



//GENERAL SERVER FUNCTIONING STUFF----------------------------------------
var FullLog=false; //log on the console in depth information
var IsPrinting=false; //is/should currently executing a gcode

//containers for the gcode that needs to be buffered
var FileGcode=[]; //every Gcode of the loaded Gcode file
var CustomBuffer=[]; //buffer of custom Gcode added

var FloatingBuffer=0; //how many not yet confirmed gcodes have been sent
var MaxFloatingBuffer=4; //max
var LastBufferIndex=-1; //the index of the last gcode added to the buffer
var LastFinishedIndex=-1; //index of the last file gcode completed

function RoundTo3(x){
  return Math.round(x*1000)/1000;
}
//returns the value of a chosen attribute in the given string
function GcodeParser(code,attr){
  var n=code.indexOf(attr);
  var val="";
  if(n!=-1){
    while(!isNaN(parseInt(code.charAt(n+1))) || code.charAt(n+1)=='-' || code.charAt(n+1)=='.'){
      val+=code.charAt(n+1);
      n++;
    }
    if(val.length==0){val="0"}
  }
  return parseFloat(val);
}
//print various useful info about the Gcode file
function FileInfo(){
  //record all code types
  var codes=[];
  var times=[]; //number of occurances of the code
  var firstindex=[]; //first appearence of every code
  var lastindex=[]; //last appearence of every code
  var attr=[]; //for each code, store an array of its possible attributes
  var svals=[]; //values of encountered S attributes
  var t=""; //the code
  var n=-1; //the index of the code in the codes array
  var m=-1; //the index of the attribute in the attr[n] array
  for(var i=0; i<FileGcode.length; i++){
    t=FileGcode[i].charAt(0);
    t+=GcodeParser(FileGcode[i],t);
    n=codes.indexOf(t);
    //create new code if t is new
    if(n==-1){
      codes.push(t);
      times.push(1);
      firstindex.push(i);
      lastindex.push(i);
      attr.push([]);
      svals.push([]);
    }
    else{ times[n]++; lastindex[n]=i;}
    //now find every attribute
    n=codes.indexOf(t);
    for(var j=2; j<FileGcode[i].length-1; j++){
      if(FileGcode[i].charAt(j)==' ' && FileGcode[i].charAt(j+1)!=' '){
        m=attr[n].indexOf(FileGcode[i].charAt(j+1));
        if(m==-1){ attr[n].push(""+FileGcode[i].charAt(j+1)); }
        if(FileGcode[i].charAt(j+1)=='S'){ svals[n].push(""+GcodeParser(FileGcode[i],'S')); }
        j++;
      }
    }
  }
  for(var i=0; i<codes.length; i++){
    codes[i]+=", times: "+times[i]+", index: "+firstindex[i]+"/"+lastindex[i]+", attr:";
    for(var j=0; j<attr[i].length; j++){
      codes[i]+=" "+attr[i][j];
    }
    if(attr[i][0]=='S'){ for(var j=0; j<svals[i].length; j++){
      codes[i]+="/"+svals[i][j];
    }}
  }
  console.log("every code type present:");
  console.log(codes);
}
//read the gcode file and create the FileGcode variable
function LoadGcode(filename){
  var str=fs.readFileSync("Gfiles/"+filename,"utf8");
  FileGcode=str.split('\n');
  for(var i=FileGcode.length-1; i>=0; i--){
    var l=FileGcode[i].indexOf(";");
    if(l>=0){
      FileGcode[i]=FileGcode[i].slice(0,l);
    }
    if(FileGcode[i].length==0){
      FileGcode.splice(i,1);
    }
  }

  //FileGcode.splice(2000,70000)

  console.log("scanning filegcode, length: "+FileGcode.length);
  //given the current polar printer type, must correct some movements
  var x=0; var y=0; var z=0; var e=0;
  var xtarg=0; var ytarg=0; var ztarg=0; var etarg=0;
  var g=-1; var rel=false; var erel=false;
  var d=0; var minD=17; //min magnitude for resulting segments for the cuts to take place
  var toreplace=[];
  for(var i=0; i<FileGcode.length; i++){
    g=GcodeParser(FileGcode[i],"G");
    if(g==1 || g==0){
      if(!rel){
        xtarg=GcodeParser(FileGcode[i],"X");
        ytarg=GcodeParser(FileGcode[i],"Y");
        ztarg=GcodeParser(FileGcode[i],"Z");
      }else{
        xtarg=GcodeParser(FileGcode[i],"X")+x;
        ytarg=GcodeParser(FileGcode[i],"Y")+y;
        ztarg=GcodeParser(FileGcode[i],"Z")+z;
      }
      if(isNaN(xtarg)){xtarg=x}
      if(isNaN(ytarg)){ytarg=y}
      if(isNaN(ztarg)){ztarg=z}
      if(!erel){etarg=GcodeParser(FileGcode[i],"E")}
      else{etarg=GcodeParser(FileGcode[i],"E")+e}
      if(isNaN(etarg)){etarg=e}
      if(!isNaN(GcodeParser(FileGcode[i],"F"))){f=GcodeParser(FileGcode[i],"F")}

      d=Math.sqrt((xtarg-x)*(xtarg-x)+(ytarg-y)*(ytarg-y));
      //if segment is long and not a travel move, break it up
      if(d>minD && etarg!=e){
        if(d/2<minD){
          toreplace.push(("G"+g+" F"+f)+(rel?(" X"+RoundTo3((xtarg-x)/2)+" Y"+RoundTo3((ytarg-y)/2)+" Z"+(ztarg-z)):(" X"+RoundTo3((xtarg+x)/2)+" Y"+RoundTo3((ytarg+y)/2)+" Z"+ztarg)))
          toreplace[0]+=(" E"+(erel?RoundTo3((etarg-e)/2):RoundTo3((etarg+e)/2)))
          toreplace.push(("G"+g)+(rel?(" X"+RoundTo3((xtarg-x)/2)+" Y"+RoundTo3((ytarg-y)/2)):(" X"+xtarg+" Y"+ytarg)))
          toreplace[1]+=(" E"+(erel?RoundTo3((etarg-e)/2):etarg))
        }
        else if(d/4<minD){
          toreplace.push(("G"+g+" F"+f)+(rel?(" X"+RoundTo3((xtarg-x)/4)+" Y"+RoundTo3((ytarg-y)/4)+" Z"+(ztarg-z)):(" X"+RoundTo3((xtarg+x*3)/4)+" Y"+RoundTo3((ytarg+y*3)/4)+" Z"+ztarg)))
          toreplace[0]+=(" E"+(erel?RoundTo3((etarg-e)/4):RoundTo3((etarg+e*3)/4)))
          toreplace.push(("G"+g)+(rel?(" X"+RoundTo3((xtarg-x)/4)+" Y"+RoundTo3((ytarg-y)/4)):(" X"+RoundTo3((xtarg*2+x*2)/4)+" Y"+RoundTo3((ytarg*2+y*2)/4))))
          toreplace[1]+=(" E"+(erel?RoundTo3((etarg-e)/4):RoundTo3((etarg*2+e*2)/4)))
          toreplace.push(("G"+g)+(rel?(" X"+RoundTo3((xtarg-x)/4)+" Y"+RoundTo3((ytarg-y)/4)):(" X"+RoundTo3((xtarg*3+x*1)/4)+" Y"+RoundTo3((ytarg*3+y*1)/4))))
          toreplace[2]+=(" E"+(erel?RoundTo3((etarg-e)/4):RoundTo3((etarg*3+e*1)/4)))
          toreplace.push(("G"+g)+(rel?(" X"+RoundTo3((xtarg-x)/4)+" Y"+RoundTo3((ytarg-y)/4)):(" X"+xtarg+" Y"+ytarg)))
          toreplace[3]+=(" E"+(erel?RoundTo3((etarg-e)/4):etarg))
        }
        else{
          toreplace.push(("G"+g+" F"+f)+(rel?(" X"+RoundTo3((xtarg-x)/6)+" Y"+RoundTo3((ytarg-y)/6)+" Z"+(ztarg-z)):(" X"+RoundTo3((xtarg+x*5)/6)+" Y"+RoundTo3((ytarg+y*5)/6)+" Z"+ztarg)))
          toreplace[0]+=(" E"+(erel?RoundTo3((etarg-e)/6):RoundTo3((etarg+e*5)/6)))
          toreplace.push(("G"+g)+(rel?(" X"+RoundTo3((xtarg-x)/6)+" Y"+RoundTo3((ytarg-y)/6)):(" X"+RoundTo3((xtarg*2+x*4)/6)+" Y"+RoundTo3((ytarg*2+y*4)/6))))
          toreplace[1]+=(" E"+(erel?RoundTo3((etarg-e)/6):RoundTo3((etarg*2+e*4)/6)))
          toreplace.push(("G"+g)+(rel?(" X"+RoundTo3((xtarg-x)/6)+" Y"+RoundTo3((ytarg-y)/6)):(" X"+RoundTo3((xtarg*3+x*3)/6)+" Y"+RoundTo3((ytarg*3+y*3)/6))))
          toreplace[2]+=(" E"+(erel?RoundTo3((etarg-e)/6):RoundTo3((etarg*3+e*3)/6)))
          toreplace.push(("G"+g)+(rel?(" X"+RoundTo3((xtarg-x)/6)+" Y"+RoundTo3((ytarg-y)/6)):(" X"+RoundTo3((xtarg*4+x*2)/6)+" Y"+RoundTo3((ytarg*4+y*2)/6))))
          toreplace[2]+=(" E"+(erel?RoundTo3((etarg-e)/6):RoundTo3((etarg*4+e*2)/6)))
          toreplace.push(("G"+g)+(rel?(" X"+RoundTo3((xtarg-x)/6)+" Y"+RoundTo3((ytarg-y)/6)):(" X"+RoundTo3((xtarg*5+x*1)/6)+" Y"+RoundTo3((ytarg*5+y*1)/6))))
          toreplace[2]+=(" E"+(erel?RoundTo3((etarg-e)/6):RoundTo3((etarg*5+e*1)/6)))
          toreplace.push(("G"+g)+(rel?(" X"+RoundTo3((xtarg-x)/6)+" Y"+RoundTo3((ytarg-y)/6)):(" X"+xtarg+" Y"+ytarg)))
          toreplace[3]+=(" E"+(erel?RoundTo3((etarg-e)/6):etarg))
        }
      }
      if(toreplace.length>0){
        FileGcode.splice(i,1,...toreplace)
        i+=toreplace.length-1;
        toreplace=[];
      }
    }
    else if(g==90){rel=false}
    else if(g==91){rel==true}
    else if(GcodeParser(FileGcode[i],"M")==82){erel=false}
    else if(GcodeParser(FileGcode[i],"M")==83){erel=true}

    x=xtarg; y=ytarg; z=ztarg; e=etarg;
  }
  console.log();
  console.log("loaded Gcode file: "+filename+", length: "+FileGcode.length);

  if(FullLog){
    FileInfo();
  }
}



//CLIENT UPDATE ALGORITHMS---------------------------------------------

var ShouldUpdateState=false;
var GcodeToShift=0; //how many gcodes the arduino has completed, that the client hasnt already updated
//var MotionToShift=0; //how many motions the arduino has completed, that the client hasnt already updated
var GcodeToPush=[]; //how many gcodes the arduino has pushed, that the client hasnt already updated
//var MotionToPush=[]; //how many motions the arduino has pushed, that the client hasnt already updated

app.get("/UpdateLoop",(req,res)=>{
  var on=(ShouldUpdateState || GcodeToShift>0 || AS.MotionToShift>0 || GcodeToPush.length>0 || AS.MotionToPush.length>0);
  var data={on:on};
  if(on){
    data={
      on:on,
      updt:ShouldUpdateState,
      GShift:GcodeToShift,
      MShift:AS.MotionToShift,
      GPush:GcodeToPush,
      MPush:AS.MotionToPush,
    };
  }
  ShouldUpdateState=false;
  GcodeToShift=0; AS.MotionToShift=0;
  GcodeToPush=[]; AS.MotionToPush=[];
  res.send(data);
});



//PRINTING GCODE AND BUFFER ALGORITHM-----------------------
function SendGcode(c){
  Serial.write("{"+c+"}");
  FloatingBuffer++;
}

//add to the buffer and send the next code of FileGcode
function NextBuffer(){
  if(LastBufferIndex+1<FileGcode.length){
    LastBufferIndex++;
    GcodeToPush.push({code:FileGcode[LastBufferIndex],index:LastBufferIndex});

    AS.AddBuffer(FileGcode[LastBufferIndex],LastBufferIndex);
    SendGcode(FileGcode[LastBufferIndex]);
    if(FullLog){ console.log("NextBuffer/ added buffer, sended: "+FileGcode[LastBufferIndex]+", index: "+LastBufferIndex); }
  }else{
    if(FullLog){ console.log("NextBuffer/ finished FileGcode"); }
  }
}
//add to the buffer and send the first code of CustomBuffer
function NextCustomBuffer(){
  GcodeToPush.push({code:CustomBuffer[0],index:-1});
  AS.AddBuffer(CustomBuffer[0],-1);
  SendGcode(CustomBuffer[0]);
  if(FullLog){ console.log("NextCustomBuffer/ added buffer, sended: "+CustomBuffer[0]+", index: -1"); }
  CustomBuffer.shift();
}

//finished current code procedure
function FinishedCode(){
  if(GcodeParser(AS.Buffer[AS.BufferHead],"G")!=1){ ShouldUpdateState=true; }
  GcodeToShift++;

  if(FullLog){ console.log("FinishedCode/ "+AS.Buffer[AS.BufferHead]+", index: "+AS.BufferIndex[AS.BufferHead]); }
  else{ console.log("FinishedCode/index: "+AS.BufferIndex[AS.BufferHead]);}
  AS.FinishCode();
  LastFinishedIndex++;

  if(IsPrinting){
    ReplenishBuffer();
    if(AS.BufferFilled==0){
      if(FullLog){ console.log("FinishedCode/ buffer empty"); }
      FinishedPrinting();
    }
  }
}

//check if the buffer needs to be replenished
function ReplenishBuffer(){
  if(FloatingBuffer<=MaxFloatingBuffer && AS.BufferFilled+1<AS.BufferSize){
    if(CustomBuffer.length>0){
      NextCustomBuffer();
    }else{
      NextBuffer();
    }
  }
}

//finished printing procedure
function FinishedPrinting(){
  console.log("Finished Printing");
  IsPrinting=false;

  ShouldUpdateState=true;
}




//LIVE GUI---------------------------------------------------
app.get("/UpdateSettings",(req,res)=>{
  var data={
    NozzleToR:AS.NozzleToR,
    PivotToTangent:AS.PivotToTangent,
    PivotX:AS.PivotX,
    RStepsPerMM:AS.RStepsPerMM, RMicrostep:AS.RMicrostep,
    AStepsPerMM:AS.AStepsPerMM, AMicrostep:AS.AMicrostep,
    ZStepsPerMM:AS.ZStepsPerMM, ZMicrostep:AS.ZMicrostep,
    EStepsPerMM:AS.EStepsPerMM, EMicrostep:AS.EMicrostep,
  }
  res.send(data);
});
app.get("/UpdateState",(req,res)=>{
  var data={
    IsPrinting:IsPrinting,
    X:0, Y:0, Z:0, E:0,
    PrintFan:AS.PrintFanOn?100:0,
    HotFan:AS.HotendFanOn?100:0,
    HotTemp:AS.HotendTemp,
    HotTarg:AS.HotendTempTarg,
    HotEffort:AS.HotendPDEffort,
  }
  if(AS.MotionBufferFilled>0){
    data.X=AS.MotionBuffer[AS.MotionBufferHead].XCart;
    data.Y=AS.MotionBuffer[AS.MotionBufferHead].YCart;
    data.Z=AS.MotionBuffer[AS.MotionBufferHead].ZCart;
    data.E=AS.MotionBuffer[AS.MotionBufferHead].ECart;
  }else{
    data.X=AS.XCart;
    data.Y=AS.YCart;
    data.Z=AS.ZCart;
    data.E=AS.ECart;
  }
  res.send(data);
});
app.get("/FastUpdateState",(req,res)=>{
  var data={
    X:0, Y:0, Z:0, E:0,
    HotTemp:AS.HotendTemp,
    HotEffort:AS.HotendPDEffort,
  }
  if(AS.MotionBufferHead<AS.MotionBufferFilled){
    data.X=AS.MotionBuffer[AS.MotionBufferHead].XCart;
    data.Y=AS.MotionBuffer[AS.MotionBufferHead].YCart;
    data.Z=AS.MotionBuffer[AS.MotionBufferHead].ZCart;
    data.E=AS.MotionBuffer[AS.MotionBufferHead].ECart;
  }else{
    data.X=AS.XCart;
    data.Y=AS.YCart;
    data.Z=AS.ZCart;
    data.E=AS.ECart;
  }
  res.send(data);
});
app.get("/FastUpdateTherm",(req,res)=>{
  res.send({
    HotTemp:AS.HotendTemp,
    HotEffort:AS.HotendPDEffort,
  });
});

app.get("/traslate/:x/:y/:z/:e",(req,res)=>{
  var code="G1";
  if(AS.AbsolutePositioning){
    code+=" X"+(AS.XCart+parseFloat(req.params.x))+" Y"+(AS.YCart+parseFloat(req.params.y))+" Z"+(AS.ZCart+parseFloat(req.params.z));
  }else{
    code+=" X"+req.params.x+" Y"+req.params.y+" Z"+req.params.z;
  }
  if(AS.AbsoluteExtrusion){
    code+=" E"+(AS.ECart+parseFloat(req.params.e));
  }else{
    code+=" E"+req.params.e;
  }
  CustomBuffer.push(code);
  res.send("traslate code added")
});
app.get("/homing/:n",(req,res)=>{
  var n=parseInt(req.params.n);
  var code="G28";
  if(n==1){code+=" X";}
  else if(n==2){code+=" Y";}
  else if(n==3){code+=" Z";}
  CustomBuffer.push(code);
  res.send("homing code added");
})
app.get("/setpos/:x/:y/:z/:e",(req,res)=>{
  var code="G92";
  if(req.params.x!="curr"){ code+=" X"+req.params.x; }
  if(req.params.y!="curr"){ code+=" Y"+req.params.y; }
  if(req.params.z!="curr"){ code+=" Z"+req.params.z; }
  if(req.params.e!="curr"){ code+=" E"+req.params.e; }

  CustomBuffer.push(code);
  res.send("setpos code added");
})

app.get("/TogglePrinting",(req,res)=>{
  if(IsPrinting){ IsPrinting=false; }
  else{ IsPrinting=true; for(var i=0; i<MaxFloatingBuffer-AS.BufferFilled; i++){ ReplenishBuffer(); }}
  console.log("IsPrinting set to: "+IsPrinting);
  res.send(IsPrinting?"1":"0");
});
app.get("/SendSingleCode",(req,res)=>{
  ReplenishBuffer();
  console.log("sent single code");
  res.send("sent single code");
});



//GCODE MANAGER-------------------------------------------------
app.get("/AddGcode/:c",(req,res)=>{
  CustomBuffer.push(req.params.c);
  //ReplenishBuffer();
  res.send("added custom gcode");
});

app.get("/UpdateBuffers",(req,res)=>{
  var data={
    gcode:[],
    motion:[]
  };
  var n=0;
  for(var i=AS.BufferHead; i!=AS.BufferTail; i=(i+1==AS.BufferSize)?0:(i+1)){
    data.gcode[n]={
      code:AS.Buffer[i],
      index:AS.BufferIndex[i],
    };
    n++;
  };
  n=0;
  for(var i=AS.MotionBufferHead; i<AS.MotionBufferFilled; i++){
    data.motion[n]={
      index:AS.MotionBuffer[i].index,
      dx:AS.MotionBuffer[i].XtargCart-AS.MotionBuffer[i].XCart,
      dy:AS.MotionBuffer[i].YtargCart-AS.MotionBuffer[i].YCart,
      dz:AS.MotionBuffer[i].ZtargCart-AS.MotionBuffer[i].ZCart,
      de:AS.MotionBuffer[i].EtargCart-AS.MotionBuffer[i].ECart,
    };
    n++;
  }

  res.send(data);
});


//OTHER----------------------------------------------------
app.get("/LoadGcode/:filename",(req,res)=>{
  LastBufferIndex=-1;
  LastFinishedIndex=-1;
  LoadGcode(req.params.filename+".gcode");
  res.send(""+FileGcode.length);
});

app.get("/SetLastIndex/:i",(req,res)=>{
  LastBufferIndex=parseInt(req.params.i)
  LastFinishedIndex=parseInt(req.params.i)
  res.send("ok")
})
app.get("/Startup",(req,res)=>{
  CustomBuffer.push("M83");
  CustomBuffer.push("M104 S206");
  CustomBuffer.push("M106 S255");

  res.send("ok")
})
app.get("/LastZ",(req,res)=>{
  var z=0;
  var val="";
  for(var i=LastBufferIndex; i>-1; i--){
    val=GcodeParser(FileGcode[i],"Z")
    if(!isNaN(val)){ z=val; i=-1; }
  }
  res.send("last z: "+z)
})
