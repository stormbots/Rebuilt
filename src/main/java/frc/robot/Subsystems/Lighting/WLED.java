// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Lighting;


import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Optional;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonSerializationContext;
import com.google.gson.JsonSerializer;

import edu.wpi.first.hal.util.UncleanStatusException;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Lighting.LedSegment.LedMultiRange;

public class WLED extends SubsystemBase{
  private static SerialPort serialport;
  //TODO: Move static serial into map of potential wled instances
  // private static HashMap<Port,WLED> ports = new HashMap<>();
  private static ArrayList<LedSegment> segments = new ArrayList<LedSegment>();
  private static ArrayList<Boolean> updated = new ArrayList<Boolean>();
  private static int calls;
  public WLED(Port port) {
    try{
      this.serialport = new SerialPort(115200, port);
      //TODO: This is intended to be a singleton-esque class

      //Attempt to write to the port to verify it functions
      serialport.writeString("{v:true}");
    }
    catch (UncleanStatusException e){
      //Raised if WLED is unplugged at robot boot
      /*
      Error at frc.robot.Subsystems.Lighting.Signals.<init>(Signals.java:24): 
      Unhandled exception instantiating robot edu.wpi.first.hal.SerialPortJNI 
      edu.wpi.first.hal.util.UncleanStatusException: Code: -1123. 
      HAL: The specified serial port device was not found
      */
      System.err.println("Could not capture serial port! WLED not running");
      this.serialport = null; //Mark invalid and avoid further writes
    }

    calls = 0; 
  }

  // TODO: Impliment as a proper singleton using a port fetch to support multiple modules
  // Note: This requires notable edits to getLedSegment and streamlining data flow
  // public static WLED getInstance(Port port){
  //   if(ports.containsKey(port)) return ports.get(port);
  //   var wled = new WLED(port);
  //   ports.put(port, wled);
  //   return wled;
  // }

  public LedSegment getLedSegment(int id, int start, int stop, boolean reverse){
    return new LedSegment(id, start, stop, reverse);
  }

  @Override
  public void periodic() {
    serializeSegments();
    SmartDashboard.putString("WLED/updated",updated.toString());
  }

  private static String gsonSerialize(LedSegment segment){
    GsonBuilder builder = new GsonBuilder();
    builder.registerTypeAdapter(LedMultiRange.class, new LedMultiRangeSerializer());
    builder.registerTypeAdapter(Optional.class, new OptionalSerializer());
    builder.registerTypeAdapter(CustomColor.class, new ColorSerializer());
    Gson gson = builder.create();
    String json = gson.toJson(segment.getData());
    return "{\"seg\":"+json +"}";
    
  }
  
  private static void serializeSegments(){
    if (updated.contains(true)){
      
      String json = "";
      boolean needsComma = false;
      for (int i = 0; i<segments.size(); i++){
        if(segments.get(i).getData().isUpdated()){
          if(needsComma){
            json += ",";
          }
          else{
            needsComma = true;
          }
          json += gsonSerialize(segments.get(i));
          segments.get(i).reset();
        }
      }
      calls++;
      if(serialport==null)return; // Invalid serial port! WLED not connected on boot
      serialport.writeString(json);
      SmartDashboard.putNumber("WLED/calls", calls);
      SmartDashboard.putString("WLED/json", json);
    } 
  }

  public static void registerSegment(LedSegment seg){
    segments.add(seg);
  }

  public static int registerState(boolean data){
    updated.add(data);
    return updated.size()-1;
  }

  public static void updateState(boolean data, int index){
    updated.set(index, data);
  }



  private static class LedMultiRangeSerializer implements JsonSerializer<LedMultiRange>{
    public JsonElement serialize(LedMultiRange ledRange, Type type, JsonSerializationContext jsonSerializationContext){
      JsonArray array = new JsonArray();
      for (int i = 0; i<ledRange.getLedRanges().size();i++){
        array.add(ledRange.getLedRanges().get(i).getStart());
        array.add(ledRange.getLedRanges().get(i).getStop());
        array.add(ledRange.getLedRanges().get(i).getColor().getHex());
      }
      return array;
    }
  }

  private static class OptionalSerializer implements JsonSerializer<Optional<?>> {
    public JsonElement serialize(Optional<?> optional, Type type, JsonSerializationContext jsonSerializationContext){
      return jsonSerializationContext.serialize(optional.orElse(null));
    }
  }

  private static class ColorSerializer implements JsonSerializer<CustomColor> {
    public JsonElement serialize(CustomColor color, Type type, JsonSerializationContext jsonSerializationContext){
      return jsonSerializationContext.serialize(color.getHex());
    }
  }
}


