// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Lighting;


import java.util.ArrayList;
import java.util.List;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WLED extends SubsystemBase {
  SerialPort leds = new SerialPort(115200, Port.kUSB1);
  // SerialPort led = new SerialPort(115200, Port.kUSB1);
  LedSegment seg = new LedSegment(0, 0, 60, false);

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
    setDefaultCommand(pride(seg));
  }

  @Override
  public void periodic() {
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

    private int toInteger(){
      return (255 << 24) | (r << 16) | (g << 8) | b;
    }

    public String getHex(){
      return Integer.toString(toInteger() & 0x00ffffff, 16);
    }
  }

  void setState(String state){
    leds.writeString(state);
    SmartDashboard.putString("JSON", state);
    // led.writeString(state);
  }

  public Command solidColor(LedSegment segment, CustomColor color){
    String[] key = {"id", "fx","col"};
    List<Object> data = new ArrayList<>();
    data.add(segment.id);
    data.add(0);
    data.add(color.getHex());
    return new InstantCommand(()->setState(toJSON(key, data)), this);
  }

  public Command blink(LedSegment segment, CustomColor color, int speed){
    String[] key = {"id", "fx","sx","col","col"};
    List<Object> data = new ArrayList<>();
    data.add(segment.id);
    data.add(1);
    data.add(speed);
    data.add(color.getHex());
    data.add(black.getHex());
    return new InstantCommand(()->setState(toJSON(key, data)), this);
  }

  public void stripes(LedSegment segment, int numStripes, CustomColor[] colors){
    int j = 0;
    int k = 0;
    boolean l = true;
    int fraction = segment.length/numStripes;
    if (fraction == 0){
      fraction = 1;
    }
    String[] key = new String[(numStripes*3)+1];
    key[0] = "id";
    List<Object> data = new ArrayList<>();
    data.add(segment.id);
  for (int i = 1; i<= numStripes*3; i++){
    key[i] = "i";
    if(i == numStripes*3-1){
      data.add(segment.length);
    }
    else if (i%3==0){
      data.add(colors[j].getHex());
      j++;
    }
    else{
      data.add((fraction)*k);
      if(l){
        k++;
      }
      l = !l;
    }
  }
    // return new RunCommand(()->setState(toJSON(key, data)), this).handleInterrupt(()->setState("{\"seg\":[{\"id\":"+segment.id+",\"frz\":false}]}"));
    setState(toJSON(key, data));
  }

   public Command pride(LedSegment segment){
    
    double StartTime = Timer.getFPGATimestamp();
    double duration = 5;
    CustomColor[] colors1 = {babyBlue,pink,white,pink,babyBlue};
    CustomColor[] colors2 = {red,orange,yellow,green,blue,purple};
    CustomColor[] colors3 = {yellow,white,purple,black};
    CustomColor[] colors4 = {magenta,magenta,purple,blue,blue};
    CustomColor[] colors5 = {yellow,yellow,yellow,yellow,yellow,yellow,purple,yellow,yellow,yellow,yellow,purple,yellow,yellow,yellow,yellow,yellow,yellow};
    CustomColor[] colors6 = {black,grey,white,purple};
    CustomColor[] colors7 = {green,lightGreen,white,grey,black};
    return new RunCommand(()->{
      double time = (Timer.getFPGATimestamp()-StartTime)%(duration*7);
      SmartDashboard.putNumber("Time", time);

      if ((time) < duration){
        stripes(segment, 5, colors1);
        return;
      }

      else if (time < duration*2){
        stripes(segment, 6, colors2);
        return;
      }

      else if (time < duration*3){
        stripes(segment, 4, colors3);
        return;
      }

      else if (time < duration*4){
        stripes(segment, 5, colors4);
        return;
      }

      else if (time < duration*5){
        stripes(segment, 18, colors5);
        return;
      }

      else if (time < duration*6){
        stripes(segment, 4, colors6);
        return;
      }

      else {
        stripes(segment, 5, colors7);
        return;
      }
    },this).handleInterrupt(()->setState("{\"seg\":[{\"id\":"+segment.id+",\"frz\":false}]}"));
  }

private String toJSON(String[] key, List<Object> data){
  //ADD EXCEPTION IF LEGTHS UNEQUAL
  if (key.length != data.size()){
    SmartDashboard.putString("lengthError", "true");
    return "";
  }
  JsonObject json = new JsonObject();
  JsonObject tempObject = new JsonObject();
  JsonArray colorArray = new JsonArray();
  JsonArray indexArray = new JsonArray();
  boolean col = false;
  boolean index = false;
  int indexCounter = 1;

  for (int i = 0; i < key.length; i++){
    if (key[i]== "col" && data.get(i) instanceof String){
      colorArray.add((String)data.get(i));
      col = true;
    }

    if (key[i]== "i" && (data.get(i) instanceof String || data.get(i) instanceof Number)){
      if (indexCounter % 3 == 0){
        indexArray.add((String)data.get(i));
      }
      else{
        indexArray.add((Number)data.get(i));
      }
      indexCounter++;
      index = true;
    }

    else if ((key[i]=="rev" || key[i]=="on") && data.get(i) instanceof Boolean){
      tempObject.addProperty(key[i], (Boolean)data.get(i));
    }

    else if (data.get(i) instanceof Number){
      tempObject.addProperty(key[i], (Number)data.get(i));
    }

    else{
      SmartDashboard.putString("typeError", indexArray.toString());
      return "";
    }
  }

  if (col){
    tempObject.add("col", colorArray);
  }

  if (index){
    tempObject.add("i", indexArray);
  }

  json.add("seg", tempObject);
  
  return json.toString();

}

}

