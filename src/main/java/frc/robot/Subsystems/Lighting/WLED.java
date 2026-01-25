// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Lighting;


import java.util.ArrayList;
import java.util.List;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class WLED extends SubsystemBase {
  SerialPort leds = new SerialPort(115200, Port.kUSB1);
  // SerialPort led = new SerialPort(115200, Port.kUSB1);
  LedSegment seg = new LedSegment(0, 0, 15, false);
  CustomColor r = new CustomColor(0, 255, 0);
  CustomColor pink = new CustomColor(255, 28, 206);
  CustomColor white = new CustomColor(255, 255, 255);
  CustomColor babyBlue = new CustomColor(38, 14, 255);
  CustomColor red = new CustomColor(255, 0, 0);
  CustomColor orange = new CustomColor(255, 140, 0);
  CustomColor yellow = new CustomColor(255, 215, 0);
  CustomColor green = new CustomColor(0, 255, 0);
  CustomColor blue = new CustomColor(0, 0, 255);
  CustomColor purple = new CustomColor(255, 0, 255);
  CustomColor black = new CustomColor(0, 0, 0);
  CustomColor grey = new CustomColor(119, 119, 119);
  CustomColor magenta = new CustomColor(255, 0, 111);
  CustomColor lightGreen = new CustomColor(159, 255, 133); 
// private Signals signlas = new Signals();
  // private Bling bling = new Bling();
  /** Creates a new Leds. */
  // LedSegment segment =new LedSegment(0, 0, 30, false);
  public WLED() {
    setDefaultCommand(solidColorTest(seg, blue));
  }

  @Override
  public void periodic() {
    //{"seg":[{"id":0, "col":[000000]}]}
    SmartDashboard.putString("hex", r.getHex());
    SmartDashboard.putNumber("Legth", seg.length);
    // This method will be called once per scheduler run
  }

  class LedSegment {
    int id;
    int start;
    int stop;
    boolean reverse;
    int length;
    LedSegment(int id, int start, int stop, boolean reverse){
      this.id = id;
      this.start = start;
      this.stop = stop;
      this.reverse = reverse;
      this.length = stop - start;
      String[] key = {"id","start","stop","rev"};
      List<Object> data = new ArrayList<>();
      data.add(id);
      data.add(start);
      data.add(stop);
      data.add(reverse);

      setState(toJSON(key,data));
    }
  }

  public class CustomColor {
    int r;
    int g;
    int b;
    public CustomColor(int r, int g, int b){
      this.r =r;
      this.g = g;
      this.b = b;
    }

    private String toHex(int n){
      if(n > 255){
        n = 255;
      }
      else if(n < 0){
        n = 0;
      }

      // char array to store hexadecimal number
      char []hexaDeciNum = new char[2];
 
      // counter for hexadecimal number array
      int i = 0;
      while (n != 0) {
 
        // temporary variable to store remainder
        int temp = 0;
 
        // storing remainder in temp variable.
        temp = n % 16;
 
        // check if temp < 10
        if (temp < 10) {
            hexaDeciNum[i] = (char) (temp + 48);
            i++;
        }
        else {
            hexaDeciNum[i] = (char) (temp + 55);
            i++;
        }
 
        n = n / 16;
      }
 
      String hexCode = "";
      if (i == 2) {
        hexCode+=hexaDeciNum[0];
        hexCode+=hexaDeciNum[1];
      }
      else if (i == 1) {
        hexCode = "0";
        hexCode+=hexaDeciNum[0];
      }
      else if (i == 0){
        hexCode = "00";
      }
        
      // Return the equivalent
      // hexadecimal color code
      return hexCode;
    }

    public String getHex(){
      String hexCode = "";
      hexCode += toHex(r);
      hexCode += toHex(g);
      hexCode += toHex(b);
 
      return hexCode;
    }
  }

  void setState(String state){
    leds.writeString(state);
    SmartDashboard.putString("JSON", state);
    // led.writeString(state);
  }

  public Command solidColorTest(LedSegment segment, CustomColor color){
    String[] keys = {"id", "fx","col"};
    List<Object> data = new ArrayList<>();
    data.add(segment.id);
    data.add(0);
    data.add(color.getHex());
    return new InstantCommand(()->setState(toJSON(keys, data)), this);
  }

  public Command blink(LedSegment segment, CustomColor color, int speed, double timeSec){
    return new InstantCommand(()->setState(
      "{\"seg\":["+
      "{"+
        "\"id\":" + segment.id + ","+
        "\"fx\":1,"+
        "\"sx\":"+ speed + ","+
        "\"col\":[\""+ color.getHex() +"\", 000000]"+
      "}"+
      "]}"), this).andThen(new WaitCommand(timeSec));
  }

   public Command pride(LedSegment segment){
    
    double StartTime = Timer.getFPGATimestamp();
    double duration = 5;
    
    return new RunCommand(()->{
      double time = (Timer.getFPGATimestamp()-StartTime)%(duration*7);
      SmartDashboard.putNumber("Time", time);
      if (time < duration){
        setState(
        "{\"seg\":["+
        "{"+
          "\"id\":" + segment.id + ","+
          "\"i\":[0,"+segment.length/5+",\""+ 
          babyBlue.getHex() +"\","+ 
          segment.length/5+","+ (segment.length/5)*2 +",\""+
          pink.getHex() +"\","+
          (segment.length/5)*2+","+(segment.length/5)*3+",\""+
          white.getHex() +"\","+
          (segment.length/5)*3+","+(segment.length/5)*4+",\""+
          pink.getHex() +"\","+
          (segment.length/5)*4+","+segment.length+",\""+
          babyBlue.getHex() +
          "\"]"+
        "}"+
        "]}");
      }

      else if (time < duration*2){
        setState(
        "{\"seg\":["+
        "{"+
          "\"id\":" + segment.id + ","+
          "\"i\":[0,"+segment.length/6+",\""+ 
          red.getHex() +"\","+ 
          segment.length/6+","+ (segment.length/6)*2 +",\""+
          orange.getHex() +"\","+
          (segment.length/6)*2+","+(segment.length/6)*3+",\""+
          yellow.getHex() +"\","+
          (segment.length/6)*3+","+(segment.length/6)*4+",\""+
          green.getHex() +"\","+
          (segment.length/6)*4+","+(segment.length/6)*5+",\""+
          blue.getHex() +"\","+
          (segment.length/6)*5+","+segment.length + ",\""+
          purple.getHex() +
          "\"]"+
        "}"+
        "]}");
      }

      else if (time < duration*3){
        setState(
        "{\"seg\":["+
        "{"+
          "\"id\":" + segment.id + ","+
          "\"i\":[0,"+segment.length/4+",\""+ 
          yellow.getHex() +"\","+ 
          segment.length/4+","+ (segment.length/4)*2 +",\""+
          white.getHex() +"\","+
          (segment.length/4)*2+","+(segment.length/4)*3+",\""+
          purple.getHex() +"\","+
          (segment.length/4)*3+","+segment.length+",\""+
          black.getHex() +
          "\"]"+
        "}"+
        "]}");
      }

      else if (time < duration*4){
        setState(
        "{\"seg\":["+
        "{"+
          "\"id\":" + segment.id + ","+
          "\"i\":[0,"+(segment.length/5)*2+",\""+ 
          magenta.getHex() +"\","+ 
          (segment.length/5)*2+","+(segment.length/5)*3+",\""+
          purple.getHex() +"\","+
          (segment.length/5)*3+","+segment.length+",\""+
          blue.getHex() +
          "\"]"+
        "}"+
        "]}");
      }

      else if (time < duration*5){
        int fraction = segment.length/18;
        if (fraction == 0){
          fraction = 1;
        }
        setState(
        "{\"seg\":["+
        "{"+
          "\"id\":" + segment.id + ","+
          "\"i\":[0,"+segment.length/3+",\""+ 
          yellow.getHex() +"\","+ 
          segment.length/3+","+ fraction*7 +",\""+
          purple.getHex() +"\","+
          fraction*7+","+fraction*11+",\""+
          yellow.getHex() +"\","+
          fraction*11+","+(segment.length/3)*2+",\""+
          purple.getHex() +"\","+
          (segment.length/3)*2+","+segment.length+",\""+
          yellow.getHex() +
          "\"]"+
        "}"+
        "]}");
      }

      else if (time < duration*6){
        setState(
        "{\"seg\":["+
        "{"+
          "\"id\":" + segment.id + ","+
          "\"i\":[0,"+segment.length/4+",\""+ 
          black.getHex() +"\","+ 
          segment.length/4+","+ (segment.length/4)*2 +",\""+
          grey.getHex() +"\","+
          (segment.length/4)*2+","+(segment.length/4)*3+",\""+
          white.getHex() +"\","+
          (segment.length/4)*3+","+segment.length+",\""+
          purple.getHex() +
          "\"]"+
        "}"+
        "]}");
      }

      else {
        setState(
        "{\"seg\":["+
        "{"+
          "\"id\":" + segment.id + ","+
          "\"i\":[0,"+segment.length/5+",\""+ 
          green.getHex() +"\","+ 
          segment.length/5+","+ (segment.length/5)*2 +",\""+
          lightGreen.getHex() +"\","+
          (segment.length/5)*2+","+(segment.length/5)*3+",\""+
          white.getHex() +"\","+
          (segment.length/5)*3+","+(segment.length/5)*4+",\""+
          grey.getHex() +"\","+
          (segment.length/5)*4+","+segment.length+",\""+
          black.getHex() +
          "\"]"+
        "}"+
        "]}");
      }
    }, this).handleInterrupt(()->setState("{\"seg\":[{\"id\":"+segment.id+",\"frz\":false}]}"));
  }

private String toJSON(String[] key, List<Object> data){
  //ADD EXCEPTION IF LEGTHS UNEQUAL
  if (key.length != data.size()){
    return "";
  }
  JsonObject json = new JsonObject();
  JsonObject tempObject = new JsonObject();
  JsonArray colorArray = new JsonArray();
  // JsonArray indexArray = new JsonArray();
  boolean col = false;
  // boolean index = false;

  for (int i = 0; i < key.length; i++){
    if (key[i]== "col" && data.get(i) instanceof String){
      colorArray.add((String)data.get(i));
      col = true;
    }

    else if ((key[i]=="rev" || key[i]=="on") && data.get(i) instanceof Boolean){
      tempObject.addProperty(key[i], (Boolean)data.get(i));
    }

    else if (data.get(i) instanceof Number){
      tempObject.addProperty(key[i], (Number)data.get(i));
    }

    else{
      return "";
    }
  }

  if (col){
    tempObject.add("col", colorArray);
  }

  json.add("seg", tempObject);
  
  return json.toString();

}

}

