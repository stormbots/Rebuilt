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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class WLED {
  static SerialPort serialport;
  // SerialPort led = new SerialPort(115200, Port.kUSB1);

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
    try{
      serialport.toString();
    }
    catch (NullPointerException n){
      serialport = new SerialPort(115200, Port.kUSB1);
    }
     
  }

  public LedSegment getLedSegment(int id, int start, int stop, boolean reverse){
    return new LedSegment(id, start, stop, reverse);
  }

  public CustomColor geCustomColor(int r, int g, int b){
    return new CustomColor(r, g, b);
  }


  public class LedSegment extends SubsystemBase {
    int id;
    int start;
    int stop;
    boolean reverse;
    int length;
    LedSegment(int id, int start, int stop, boolean reverse){
      this.id = id;
      this.start = start;
      this.stop = stop;
      this.length = stop - start;
      String[] key = {"id","start","stop","rev"};
      List<Object> data = new ArrayList<>();
      data.add(id);
      data.add(start);
      data.add(stop);
      data.add(reverse);

      writeJSON(toJSON(key,data));
    }

    void writeJSON(String state){
    serialport.writeString(state);
    SmartDashboard.putString("JSON", state);
    // led.writeString(state);
  }


  public Command solidColor(CustomColor color){
    String[] key = {"id","fx","col"};
    List<Object> data = new ArrayList<>();
    data.add(id);
    data.add(0);
    data.add(color.getHex());
    return new InstantCommand(()->writeJSON(toJSON(key, data)), this);
  }

  public Command blink(CustomColor color, int speed){
    String[] key = {"id", "fx","sx","col","col"};
    List<Object> data = new ArrayList<>();
    data.add(id);
    data.add(1);
    data.add(speed);
    data.add(color.getHex());
    data.add(black.getHex());
    return new InstantCommand(()->writeJSON(toJSON(key, data)), this);
  }

  public Command stripes(int numStripes, CustomColor[] colors){
    int j = 0;
    int k = 0;
    boolean l = true;
    int fraction = length/numStripes;
    if (fraction == 0){
      fraction = 1;
    }
    String[] key = new String[(numStripes*3)+1];
    key[0] = "id";
    List<Object> data = new ArrayList<>();
    data.add(id);
  for (int i = 1; i<= numStripes*3; i++){
    key[i] = "i";
    if(i == numStripes*3-1){
      data.add(length);
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
    return new InstantCommand(()->writeJSON(toJSON(key, data)), this);//.handleInterrupt(()->setState("{\"seg\":[{\"id\":"+segment.id+",\"frz\":false}]}"));
    // setState(toJSON(key, data));
    
  }

  public Command pride(){
    double duration = 2.5;
    CustomColor[] colors1 = {babyBlue,pink,white,pink,babyBlue};
    CustomColor[] colors2 = {red,orange,yellow,green,blue,purple};
    CustomColor[] colors3 = {yellow,white,purple,black};
    CustomColor[] colors4 = {magenta,magenta,purple,blue,blue};
    CustomColor[] colors5 = {yellow,yellow,yellow,yellow,yellow,yellow,purple,yellow,yellow,yellow,yellow,purple,yellow,yellow,yellow,yellow,yellow,yellow};
    CustomColor[] colors6 = {black,grey,white,purple};
    CustomColor[] colors7 = {green,lightGreen,white,grey,black};

    return new SequentialCommandGroup(stripes(5, colors1)
    .andThen(new WaitCommand(duration))
    .andThen(stripes( 6, colors2))
    .andThen(new WaitCommand(duration))
    .andThen(stripes(4, colors3))
    .andThen(new WaitCommand(duration))
    .andThen(stripes(5, colors4))
    .andThen(new WaitCommand(duration))
    .andThen(stripes(18, colors5))
    .andThen(new WaitCommand(duration))
    .andThen(stripes(4, colors6))
    .andThen(new WaitCommand(duration))
    .andThen(stripes(5, colors7))
    .andThen(new WaitCommand(duration))).handleInterrupt(()->writeJSON("{\"seg\":[{\"id\":"+id+",\"frz\":false}]}"));
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

      else if (key[i]== "i" && (data.get(i) instanceof String || data.get(i) instanceof Number)){
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
        SmartDashboard.putString("typeErrorColor", colorArray.toString());
        SmartDashboard.putString("typeErrorIndex", indexArray.toString());
        SmartDashboard.putString("typeError", tempObject.toString());
        SmartDashboard.putString("key", key[i]);
        SmartDashboard.putString("data",(String)data.get(i));
        SmartDashboard.putBoolean("string",data.get(i) instanceof String);
        SmartDashboard.putNumber("index", i);
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
      String hex = Integer.toString(toInteger() & 0x00ffffff, 16);
      if (r == 0){
        hex = "00"+hex;
      }
      if (g == 0 && r == 0){
        hex = "00"+hex;
      }
      return hex;
    }
  }

  

  

   

  
}

