module.exports={

  Data:[], //data currently recorded
  IsRecording:false, //if it is currently recording data

  Constants:{},

  RecordData: function(data){
    var x=parseFloat(data.slice(0,data.indexOf(",")));
    var y=parseFloat(data.slice(data.indexOf(",")+1,data.length));
    this.Data.push({X:x,Y:y});
  },
  EraseData:function(){
    this.Data=[];
  },


  //return the temperature value during cooling
  CoolingTemperature: function(t, t0, T0, Tamb){
    return ((T0-Tamb)*Math.exp(-1*this.Constants.Knewton*(t-t0))+Tamb);
  },

  //best fit for variable k in y[i]=y[0]*e^(-k*x[i])
  CalcConstants: function(fs){
    //calculate newton coefficient between T and dT
    this.Data=JSON.parse(fs.readFileSync("./MB3DThermal/HotendHeater-CoolingData.json"));
    var avrg=0;
    for(var i=1; i<this.Data.length; i++){
      avrg-=Math.log((this.Data[i].Y-this.Constants.Tamb)/(this.Data[0].Y-this.Constants.Tamb))/(this.Data[i].X-this.Data[0].X);
    }
    this.Constants.Knewton=avrg/(this.Data.length-1);

    //calculate K during heating
    this.Data=JSON.parse(fs.readFileSync("./MB3DThermal/HotendHeater-10HeatingData.json"));
    var avrg1=0;
    for(var i=1; i<this.Data.length; i++){
      avrg1+=this.Constants.Knewton*(this.Data[i].Y-this.Constants.Tamb)/(1-Math.exp(-this.Constants.Knewton*(this.Data[i].X-this.Data[0].X)));
    }
    avrg1*=10/(this.Data.length-1);
    this.Data=JSON.parse(fs.readFileSync("./MB3DThermal/HotendHeater-15HeatingData.json"));
    var avrg2=0;
    for(var i=1; i<this.Data.length; i++){
      avrg2+=this.Constants.Knewton*(this.Data[i].Y-this.Constants.Tamb)/(1-Math.exp(-this.Constants.Knewton*(this.Data[i].X-this.Data[0].X)));
    }
    avrg2*=6.6666/(this.Data.length-1);
    this.Data=JSON.parse(fs.readFileSync("./MB3DThermal/HotendHeater-20HeatingData.json"));
    var avrg3=0;
    for(var i=1; i<this.Data.length; i++){
      avrg3+=this.Constants.Knewton*(this.Data[i].Y-this.Constants.Tamb)/(1-Math.exp(-this.Constants.Knewton*(this.Data[i].X-this.Data[0].X)));
    }
    avrg3*=5/(this.Data.length-1);
    this.Data=JSON.parse(fs.readFileSync("./MB3DThermal/HotendHeater-30HeatingData.json"));
    var avrg4=0;
    for(var i=1; i<this.Data.length; i++){
      avrg4+=this.Constants.Knewton*(this.Data[i].Y-this.Constants.Tamb)/(1-Math.exp(-this.Constants.Knewton*(this.Data[i].X-this.Data[0].X)));
    }
    avrg4*=3.3333/(this.Data.length-1);
    this.Constants.Kheating=(avrg1+avrg2+avrg3+avrg4)/4;

    //calculate newton coefficient between T and dT, with 100% fan
    this.Data=JSON.parse(fs.readFileSync("./MB3DThermal/HotendHeater-CoolingFanData.json"));
    avrg=0;
    for(var i=1; i<this.Data.length; i++){
      avrg-=Math.log((this.Data[i].Y-this.Constants.Tamb)/(this.Data[0].Y-this.Constants.Tamb))/(this.Data[i].X-this.Data[0].X);
    }
    //fan coefficient is how much the fan augments the passive newton coefficient
    this.Constants.KnewtonFan=avrg/(this.Data.length-1);
    //calculate K during heating
    this.Data=JSON.parse(fs.readFileSync("./MB3DThermal/HotendHeater-10HeatingFanData.json"));
    avrg1=0;
    for(var i=1; i<this.Data.length; i++){
      avrg1+=this.Constants.KnewtonFan*(this.Data[i].Y-this.Constants.Tamb)/(1-Math.exp(-this.Constants.KnewtonFan*(this.Data[i].X-this.Data[0].X)));
    }
    avrg1*=10/(this.Data.length-1);
    this.Data=JSON.parse(fs.readFileSync("./MB3DThermal/HotendHeater-15HeatingFanData.json"));
    avrg2=0;
    for(var i=1; i<this.Data.length; i++){
      avrg2+=this.Constants.KnewtonFan*(this.Data[i].Y-this.Constants.Tamb)/(1-Math.exp(-this.Constants.KnewtonFan*(this.Data[i].X-this.Data[0].X)));
    }
    avrg2*=6.6666/(this.Data.length-1);
    this.Data=JSON.parse(fs.readFileSync("./MB3DThermal/HotendHeater-20HeatingFanData.json"));
    avrg3=0;
    for(var i=1; i<this.Data.length; i++){
      avrg3+=this.Constants.KnewtonFan*(this.Data[i].Y-this.Constants.Tamb)/(1-Math.exp(-this.Constants.KnewtonFan*(this.Data[i].X-this.Data[0].X)));
    }
    avrg3*=5/(this.Data.length-1);
    this.Data=JSON.parse(fs.readFileSync("./MB3DThermal/HotendHeater-30HeatingFanData.json"));
    avrg4=0;
    for(var i=1; i<this.Data.length; i++){
      avrg4+=this.Constants.KnewtonFan*(this.Data[i].Y-this.Constants.Tamb)/(1-Math.exp(-this.Constants.KnewtonFan*(this.Data[i].X-this.Data[0].X)));
    }
    avrg4*=3.3333/(this.Data.length-1);
    this.Constants.KheatingFan=(avrg1+avrg2+avrg3+avrg4)/4;

    fs.writeFileSync("./MB3DThermal/Constants.json", JSON.stringify(this.Constants, null, 2));

    console.log("mb3dThermal/calculated constants");
  },


}
