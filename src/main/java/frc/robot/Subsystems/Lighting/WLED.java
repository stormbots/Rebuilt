// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Lighting;


import java.util.ArrayList;
import java.util.List;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class WLED {
  static SerialPort serialport;
  // SerialPort led = new SerialPort(115200, Port.kUSB1);
  int calls;
  
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
    calls = 0; 
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
    String[] lastKey;
    String lastState; //for debug
    List<Object> lastData;
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

      writeJSONUnchecked(key,data);
    }

    void writeJSON(String[] key, List<Object> data){
      String state = lastState;
      if (differentData(key, data)){
        state = toJSON(key, data);
        lastKey = key;
        lastData = data;
        lastState = state;
        if (!state.equals("" )){
          serialport.writeString(state);
          calls++;
        }
      }
      SmartDashboard.putString("JSON", state);
      SmartDashboard.putNumber("calls", calls);
      SmartDashboard.putBoolean("different", differentData(key, data));
  }

  void writeJSONUnchecked(String state){
      lastKey = null;
      lastData = null;
      lastState = state;
      serialport.writeString(state);
      SmartDashboard.putString("JSON", state);
      SmartDashboard.putNumber("calls", calls);
      calls++;
  }

  void writeJSONUnchecked(String[] key, List<Object> data){
      lastKey = key;
      lastData = data;
      String state;
      if (key.length != data.size()){
        SmartDashboard.putString("lengthError", "true");
        state = "";
      }
      else{
        state = toJSON(key, data);
      }
      if (!state.equals("")){
        serialport.writeString(state);
        calls++;
      }
      lastState = state;
      
      SmartDashboard.putString("JSON", state);
      SmartDashboard.putNumber("calls", calls);
      
  }

  private boolean differentData(String[] key, List<Object> data){
    if(key == null || data == null){
      return true;
    }
    if (key.length != data.size()){
      SmartDashboard.putString("lengthError", "true");
      SmartDashboard.putNumber("keyLength", key.length);
      SmartDashboard.putNumber("dataLength", data.size());
      SmartDashboard.putString("data", data.toString());
      return false;
    }
    if(key.length != lastKey.length){
      return true;
    }
    else{
      for (int i = 0; i < key.length; i++){
        if(!key[i].equals(lastKey[i])){
          return true;
        }
        else{
          if(data.get(i) instanceof Boolean && lastData.get(i) instanceof Boolean){
            if(data.get(i)!= lastData.get(i)){
              return true;
            }
          }
          else if(data.get(i) instanceof Number && lastData.get(i) instanceof Number){
            if(data.get(i)!= lastData.get(i)){
              return true;
            }
          }
          else if(data.get(i) instanceof String && lastData.get(i) instanceof String){
            if(data.get(i)!= lastData.get(i)){
              return true;
            }
          }
          else{
            return true;
          }
        }
      }
    }
    return false;
  }


  public Command solidColor(CustomColor color){
    String[] key = {"id","fx","col"};
    List<Object> data = new ArrayList<>();
    data.add(id);
    data.add(0);
    data.add(color.getHex());
    return new InstantCommand(()->writeJSON(key, data), this);
  }

  public Command blink(CustomColor color, int speed){
    String[] key = {"id", "fx","sx","col","col"};
    List<Object> data = new ArrayList<>();
    data.add(id);
    data.add(1);
    data.add(speed);
    data.add(color.getHex());
    data.add(black.getHex());
    return new InstantCommand(()->writeJSON(key, data), this);
  }

  public Command stripes(CustomColor[] colors){
    int j = 0;
    int numStripes = colors.length;
    int l = 0;
    int num = 0;
    boolean addColor;
    boolean addNumber;
    boolean end = false;
    boolean sameLastColor = false;
    boolean sameNextColor = false;
    int fraction = length/numStripes;
    if (fraction == 0){
      fraction = 1;
    }
    for (int i=0; i<colors.length; i++){
      if(i==0){
        num++;
      }
      else if (colors[i] != colors[i-1]){
        num ++;
      }
    }
    
    int instances =1;
    String[] key = new String[(num*3)+1];
    key[0] = "id";
    List<Object> data = new ArrayList<>();
    data.add(id);
  for (int i = 0; i< numStripes*3; i++){
    if (j!=0){
      sameLastColor = (colors[Math.min(j, colors.length-1)]==colors[Math.min(j-1, colors.length-2)]);
    }
    
    sameNextColor = (colors[Math.min(j, colors.length-2)]==colors[Math.min(j+1, colors.length-1)]);
    
    
    addNumber = l < 2;
    addNumber = addNumber || (data.get(l) instanceof String);
    addNumber = addNumber || ((data.get(l) instanceof Number) && (data.get(l-1) instanceof String));
    addNumber = addNumber && l<key.length-2;
    addColor = !addNumber && l<=key.length-2;
    
    
    if (addColor || end){
      // data.add("j"+Integer.toString(j));
      data.add(colors[j].getHex());
      j++;
      if (end){
        end =false;
      }
      l++;
      
      key[l] = "i";
    }
    else if (addNumber) {

      if(l==key.length-3){
        // data.add(76);
        data.add(length);
        l++;
        key[l] = "i";
        end = true;
      }
      else if (l == 0){
          // data.add(j);
          data.add(0);
          l++;
          instances++;
          key[l] = "i";
          if (sameNextColor){
            j++;
          }
          // j++;
        }

      else if (!sameLastColor && sameNextColor){
        
        // data.add(25);
        data.add((int)data.get(l-1));
        l++;
        key[l] = "i";
        j++;
        instances++;
      }

      else if(sameLastColor && !sameNextColor){
        int newFraction = (int)(length * ((double)instances/numStripes));
        if (newFraction == 0){
          newFraction = 1;
        }
        // data.add(77);
        data.add((int)data.get(l)+newFraction);
        l++;
        key[l] = "i";
        instances = 1;
        // j++;
      }

      else if (!sameLastColor && !sameNextColor){
        if(data.get(l) instanceof Number){
          data.add((int)data.get(l)+fraction);
          // data.add(67);
        }
        else{
          data.add((int)data.get(l-1));
          // data.add(52);
        }
        l++;
        key[l] = "i";
        }
      else if (sameNextColor && sameLastColor){
        // data.add(addNumber);
        instances++;
        j++;
      }
    
    }
    // else{
    //   data.add(67);
    // }
  }
    SmartDashboard.putString("data", data.toString());
    return new InstantCommand(()->writeJSON(key, data), this);//.handleInterrupt(()->setState("{\"seg\":[{\"id\":"+segment.id+",\"frz\":false}]}"));
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

    return new SequentialCommandGroup(stripes(colors1)
    .andThen(new WaitCommand(duration))
    .andThen(stripes(colors2))
    .andThen(new WaitCommand(duration))
    .andThen(stripes(colors3))
    .andThen(new WaitCommand(duration))
    .andThen(stripes(colors4))
    .andThen(new WaitCommand(duration))
    .andThen(stripes(colors5))
    .andThen(new WaitCommand(duration))
    .andThen(stripes(colors6))
    .andThen(new WaitCommand(duration))
    .andThen(stripes(colors7))
    .andThen(new WaitCommand(duration))).handleInterrupt(()->writeJSONUnchecked("{\"seg\":[{\"id\":"+id+",\"frz\":false}]}"));
  }

  private String toJSON(String[] key, List<Object> data){
    //ADD EXCEPTION IF LEGTHS UNEQUAL
    // if (key.length != data.size()){
    //   SmartDashboard.putString("lengthError", "true");
    //   SmartDashboard.putNumber("keyLength", key.length);
    //   SmartDashboard.putNumber("dataLength", data.size());
    //   SmartDashboard.putString("data", data.toString());
    //   return "";
    // }
    JsonObject json = new JsonObject();
    JsonObject tempObject = new JsonObject();
    JsonArray colorArray = new JsonArray();
    JsonArray indexArray = new JsonArray();
    boolean col = false;
    boolean index = false;
    
    for (int i = 0; i < key.length; i++){
      if (key[i]== "col" && data.get(i) instanceof String){
        colorArray.add((String)data.get(i));
        col = true;
      }

      else if (key[i]== "i" && (data.get(i) instanceof String || data.get(i) instanceof Number)){
        if (data.get(i) instanceof String){
          indexArray.add((String)data.get(i));
        }
        else{
          indexArray.add((Number)data.get(i));
        }
        index = true;
      }

      else if ((key[i]=="rev" || key[i]=="on") && data.get(i) instanceof Boolean){
        tempObject.addProperty(key[i], (Boolean)data.get(i));
      }

      else if (data.get(i) instanceof Number){
        tempObject.addProperty(key[i], (Number)data.get(i));
      }

      else{
        // SmartDashboard.putString("typeErrorColor", colorArray.toString());
        // SmartDashboard.putString("typeErrorIndex", indexArray.toString());
        // SmartDashboard.putString("typeError", tempObject.toString());
        // // SmartDashboard.putString("key", key[i]);
        // SmartDashboard.putString("data",(String)data.get(i));
        // SmartDashboard.putBoolean("string",data.get(i) instanceof String);
        // SmartDashboard.putNumber("index", i);
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

