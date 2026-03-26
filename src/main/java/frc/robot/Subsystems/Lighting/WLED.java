// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Lighting;


import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.Optional;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonSerializationContext;
import com.google.gson.JsonSerializer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Lighting.LedSegment.LedMultiRange;
import frc.robot.Subsystems.Photonvision.Photonvision;

public class WLED extends SubsystemBase{
  private static SerialPort serialport;
  public Signals signals = new Signals();
  private static Photonvision vision;
  private static ArrayList<LedSegment> segments = new ArrayList<LedSegment>();
  private static ArrayList<Boolean> updated = new ArrayList<Boolean>();
  private static int calls;
  private static int defaultPattern = -1;
  private static int indivualRoll = -1;
  public static final int numPatterns = 23;
  private static int chanceIndividual = 8;
  private static boolean auraMode = false;
  public WLED(SerialPort serialPort, Photonvision vision) {
    if (chanceIndividual==0){
      chanceIndividual = 2;
    }
    try{
      serialport.toString();
    }
    catch (NullPointerException n){
      WLED.serialport = serialPort;
    }

    if (defaultPattern == -1){
      defaultPattern = (int)(Math.random()*(numPatterns));
    }

    if (indivualRoll == -1){
      indivualRoll = (int)(Math.random()*(chanceIndividual*2)+1);
    }
    calls = 0; 
    
    try{
      WLED.vision.hasTarget();
    }
    catch (NullPointerException n){
      WLED.vision = vision;
    }
     
}

  public LedSegment getLedSegment(int id, int start, int stop, boolean reverse){
    return new LedSegment(id, start, stop, reverse);
  }

  public static Command setAuraMode(){
    return Commands.runOnce(()->{
      if (!auraMode){
        auraMode = true;
      }
    });
  }


  public static int getDefaultPattern(){
    if (!DriverStation.isDSAttached()){
      return numPatterns+2;
    }
    else if(DriverStation.isAutonomousEnabled()){
      return defaultPattern;
    }
    else if (DriverStation.isTeleopEnabled()){
      return numPatterns+3;
    }
    else if (vision.doesNotHaveTarget()&&!auraMode){
      return numPatterns+4;
    }
    else if(indivualRoll == (chanceIndividual*2)-1){
      return numPatterns;
    }
    else if(indivualRoll == chanceIndividual*2){
      return numPatterns+1;
    }
    else{
      return defaultPattern;
    }
      
  }
  
  public static int getDefaultPatternUnchecked(){
    return defaultPattern;
  }

  @Override
  public void periodic() {
    serializeSegments();
    // SmartDashboard.putNumber("leds/defaultPattern", getDefaultPattern());
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
          segments.get(i).resetData();
        }
      }
      calls++;
      serialport.writeString(json);
      SmartDashboard.putNumber("calls", calls);
      SmartDashboard.putString("json", json);
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


