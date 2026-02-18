// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Lighting;


import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonSerializationContext;
import com.google.gson.JsonSerializer;

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
  ArrayList<LedSegment> segments;
  int calls;
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

  public class LedSegmentData{
    private int id; //set once
    private Optional<Integer> start;// set once
    private Optional<Integer> stop; //set once
    private CustomColor[] col;
    private Optional<Integer> fx;
    private Optional<Integer> sx;
    private Optional<Integer> ix;
    private Optional<Integer> c1;
    private Optional<Integer> c2;
    private Optional<Integer> c3;
    private Optional<Boolean> rev;
    private Optional<Boolean> on;
    private Optional<Integer> bri;
    private frc.robot.Subsystems.Lighting.WLED.LedSegment.LedMultiRange i;
    private Optional<Boolean> frz;
    private transient boolean updated;

    private void setID(int id){
      this.id = id;
      updated  = true;
    }

    private void setStart(int start){
      this.start = Optional.of(start);
      updated  = true;
    }

    private void setStop(int stop){
      this.stop = Optional.of(stop);
      updated  = true;
    }

    public void setColor(CustomColor... col){
      this.col = col;
      updated  = true;
    }

    public void setEffect(int fx){
      this.fx = Optional.of(fx);
      updated  = true;
    }

    public void setSpeed(int sx){
      this.sx = Optional.of(sx);
      updated  = true;
    }

    public void setIntensity(int ix){
      this.ix = Optional.of(ix);
      updated  = true;
    }
    
    public void setCustomSlider1(int c1){
      this.c1 = Optional.of(c1);
      updated  = true;
    }

    public void setCustomSlider2(int c2){
      this.c2 = Optional.of(c2);
      updated  = true;
    }

    public void setCustomSlider3(int c3){
      this.c3 = Optional.of(c3);
      updated  = true;
    }

    public void setReverse(boolean rev){
      this.rev = Optional.of(rev);
      updated  = true;
    }

    public void setOn(boolean on){
      this.on = Optional.of(on);
      updated  = true;
    }

    public void setBrightness(int bri){
      this.bri = Optional.of(bri);
      updated  = true;
    }

    public void setIndividualControl(frc.robot.Subsystems.Lighting.WLED.LedSegment.LedMultiRange i){
      this.i = i;
      updated  = true;
    }

    public void setFreeze(boolean frz){
      this.frz = Optional.of(frz);
      updated  = true;
    }

    public void reset(){
      start =Optional.empty();// set once
      stop = Optional.empty(); //set once
      col = null;
      fx = Optional.empty();
      sx = Optional.empty();
      ix = Optional.empty();
      c1 = Optional.empty();
      c2 = Optional.empty();
      c3 = Optional.empty();
      rev = Optional.empty();
      on = Optional.empty();
      bri = Optional.empty();
      i = null;
      frz = Optional.empty();
      updated = false;
    }

    public boolean isUpdated(){
      return updated;
    }


  }

  public class LedSegment extends SubsystemBase {
    private int id; //set once
    private int start;// set once
    private int stop; //set once
    private CustomColor[] col;
    private int fx;
    private int sx;
    private int ix;
    private int c1;
    private int c2;
    private int c3;
    private boolean rev;
    private boolean on;
    private int bri;
    private LedMultiRange i;
    private boolean frz;
    public LedSegmentData data = new LedSegmentData();
    private int length;
    private boolean[] valueUpdated = new boolean[15];
    private String[] lastKey;
    private String lastState; //for debug
    private List<Object> lastData;
    public LedSegment(int id, int start, int stop, boolean reverse){
      setID(id);
      setStart(start);
      setStop(stop);
      this.length = stop - start;
    }

    private void setID(int id){
        this.id = id;
        data.setID(id);
    }

    private void setStart(int start){
      this.start = start;
      data.setStart(start);
      
    }

    private void setStop(int stop){
      this.stop = stop;
      data.setStop(stop);
    }

    public void setColor(CustomColor... col){
      if(this.col == null ||!this.col.equals(col)){
        this.col = col;
        data.setColor(col);
        setFreeze(false);
      }
    }

    public void setEffect(int fx){
      if(this.fx != fx){
        this.fx = fx;
        data.setEffect(fx);
        setFreeze(false);
      }
    }

    public void setSpeed(int sx){
      if(this.sx != sx){
        this.sx = sx;
        data.setSpeed(sx);
        setFreeze(false);
      }
    }

    public void setIntensity(int ix){
      if(this.ix != ix){
        this.ix = ix;
        data.setIntensity(ix);
        setFreeze(false);
      }
    }
    
    public void setCustomSlider1(int c1){
      if(this.c1 != c1){
        this.c1 = c1;
        data.setCustomSlider1(c1);
        setFreeze(false);
      }
    }

    public void setCustomSlider2(int c2){
      if(this.c2 != c2){
        this.c2 = c2;
        data.setCustomSlider2(c2);
        setFreeze(false);
      }
    }

    public void setCustomSlider3(int c3){
      if(this.c3 != c3){
        this.c3 = c3;
        data.setCustomSlider3(c3);
        setFreeze(false);
      }
    }

    public void setReverse(boolean rev){
      if(this.rev != rev){
        this.rev = rev;
        data.setReverse(rev);
        setFreeze(false);
      }
    }

    public void setOn(boolean on){
      if(this.on != on){
        this.on = on;
        data.setOn(on);
        setFreeze(false);
      }
    }

    public void setBrightness(int bri){
      if(this.bri != bri){
        this.bri = bri;
        data.setBrightness(bri);
        setFreeze(false);
      }
    }

    public void setIndividualControl(LedMultiRange i){
      if(this.i == null||!this.i.equals(i)){
        this.i = i;
        data.setIndividualControl(i);
        frz = true;
      }
    }

    public void setFreeze(boolean frz){
      if(this.frz != frz){
        this.frz = frz;
        data.setFreeze(frz);
      }
    }

    public void reset(){
      data.reset();
    }

    public LedSegmentData getData(){
      return data;
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        // GsonSerialize(this);
        serialport.writeString(gsonSerialize(this));
        SmartDashboard.putString("gson", gsonSerialize(this));
        reset();
    }

    public Command solidColor(CustomColor color){
      String[] key = {color.getHex()};
      List<Object> data = new ArrayList<>();
      data.add(id);
      data.add(0);
      data.add(color.getHex());
      return new InstantCommand(()->{
        setColor(color);
        setEffect(0);
      }, this);
    }

    public Command blink(CustomColor color, int speed){
      return new InstantCommand(()->{
        setEffect(1);
        setColor(color,CustomColor.kBlack);
      }, this);
    }
    // TODO: Add compensation for unset start/stop
    private class LedRange{
      private int start;
      private int stop;
      CustomColor color;
      public LedRange(int start, int stop, CustomColor color){
        this.start =start;
        this.stop = stop;
        this.color = color;
      }

      public LedRange(CustomColor color){
        start = -1;
        stop = -1;
        this.color = color;
      }
      public int getStart(){
        return start;
      } 
      public int getStop(){
        return stop;
      }
      public CustomColor getColor(){
        return color;
      }
      public void setStart(int start){
        this.start = start;
      } 
      public void setStop(int stop){
        this.stop = stop;
      }
      public void setColor(CustomColor color){
        this.color = color;
      } 
    }
    // TODO: Add compensation for unset start/stop
    private class LedMultiRange{
      private ArrayList<LedRange> list;
      private boolean uniform = false;
      public LedMultiRange(LedRange... list){
        this.uniform = false;
        this.list = correctList(list);
      }

      public LedMultiRange(boolean uniform, LedRange... list){
        this.uniform = uniform;
        this.list = correctList(list);
        
        
      }

      private ArrayList<LedRange> correctList(LedRange[] list){
        ArrayList<LedRange> correctList = new ArrayList<LedRange>();
        if (!uniform){
          for(int i=0; i<list.length; i++){
            if (i>0 && list[i].getColor() == list[i-1].getColor()){
              correctList.get(correctList.size()-1).setStop(list[1].getStop());
            }
            else if (list[i].getStart() == -1){
              correctList.add(new LedRange(list[i].getColor()));
              if (i ==0){
                correctList.get(i).setStart(0);
              }
              else{
                correctList.get(i).setStart(correctList.get(i-1).getStop());
              }
              if (i == list.length-1){
                correctList.get(i).setStop(length);
              }
              else{
                correctList.get(i).setStop(list[i+1].getStart());
              }
              
            }
            else{
              correctList.add(new LedRange(list[i].getStart(), list[i].getStop(), list[i].getColor()));
            }
          }
          return correctList;
          
        }
        else{
          int instances = 1;
          int fraction = length/list.length;
          if (fraction == 0){
            fraction = 1;
          }

          for(int i=0; i<list.length; i++){
            if (i>0 && list[i].getColor() == list[i-1].getColor()){
              instances++;
              int newFraction = instances*length/list.length;
              if (newFraction == 0){
                newFraction = 1;
              }
              correctList.get(correctList.size()-1).setStop(correctList.get(correctList.size()-1).getStart() + newFraction);
            }
            else{
              if (i == list.length-1){
                correctList.add(new LedRange(correctList.get(i-1).getStop(), length, list[i].getColor()));
              }
              else if(i == 0){
                correctList.add(new LedRange(0, fraction, list[i].getColor()));
              }
              else{
                correctList.add(new LedRange(correctList.get(i-1).getStop(), correctList.get(i-1).getStop()+fraction, list[i].getColor()));
              }
              instances = 1;
            }
          }
          return correctList;
        }
      }

      public ArrayList<LedRange> getLedRanges(){
        return list;
      }
    }

    

    public Command stripes(LedMultiRange colors){
      return new InstantCommand(()->{
        setIndividualControl(colors);
      }, this);      
    }

    public Command pride(){
      double duration = 2.5;
      LedMultiRange colors1 = new LedMultiRange(true, new LedRange(CustomColor.kBabyBlue),new LedRange(CustomColor.kPink), new LedRange(CustomColor.kWhite),new LedRange(CustomColor.kPink),new LedRange(CustomColor.kBabyBlue));
      LedMultiRange colors2 = new LedMultiRange(true, new LedRange(CustomColor.kRed),new LedRange(CustomColor.kOrange), new LedRange(CustomColor.kYellow),new LedRange(CustomColor.kGreen),new LedRange(CustomColor.kBlue),new LedRange(CustomColor.kPurple));
      LedMultiRange colors3 = new LedMultiRange(true, new LedRange(CustomColor.kYellow),new LedRange(CustomColor.kWhite), new LedRange(CustomColor.kPurple),new LedRange(CustomColor.kBlack));
      LedMultiRange colors4 = new LedMultiRange(new LedRange(CustomColor.kMagenta),new LedRange((int)(length/5.0*2),(int)(length/5.0*3),CustomColor.kPurple), new LedRange(CustomColor.kBlue));
      LedMultiRange colors5 = new LedMultiRange(new LedRange(CustomColor.kYellow),new LedRange((int)(length/18.0*6),(int)(length/18.0*7),CustomColor.kPurple), new LedRange(CustomColor.kYellow),new LedRange((int)(length/18.0*11),(int)(length/18.0*12),CustomColor.kPurple),new LedRange(CustomColor.kYellow));
      LedMultiRange colors6 = new LedMultiRange(true, new LedRange(CustomColor.kBlack),new LedRange(CustomColor.kGrey), new LedRange(CustomColor.kWhite),new LedRange(CustomColor.kPurple));
      LedMultiRange colors7 = new LedMultiRange(true, new LedRange(CustomColor.kGreen),new LedRange(CustomColor.kLightGreen), new LedRange(CustomColor.kWhite),new LedRange(CustomColor.kGrey),new LedRange(CustomColor.kBlack));

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
      .andThen(new WaitCommand(duration)));
    }

    //seguimos aquí
    //chile {"seg":{"i":[0,8,"0000FF",8,12,"FFFFFF",12,20,"0000FF",20,40,"FFFFFF",40,60,"FF0000"]}}
    //argentina {"seg":{"i":[0,20,"6CACE4",20,25,"FFFFFF",25,35,"FFB81C",35,40,"FFFFFF",40,60,"6CACE4"]}}
    //uruguay {"seg":{"i":[0,7,"FFFFFF",7,23,"FFCD00",23,30,"FFFFFF",30,37,"0000FF",37,44,"FFFFFF",44,51,"0000FF",51,60,"FFFFFF"]}} 9 stripes
    //paraguay {"seg":{"i":[0,20,"FF0000",20,25,"FFFFFF",25,27,"1ccc00",27,28,"FFFFFF",28,32,"FFCD00",32,33,"FFFFFF",33,35,"1ccc00",35,40,"FFFFFF",40,60,"0000FF"]}}
    //bolivia {"seg":{"i":[0,20,"DA291C",20,26,"F8E600",26,29,"DA291C",29,31,"F8E600",31,34,"DA291C",34,40,"F8E600",40,60,"007A33"]}}
    //        {"seg":{"i":[0,20,"DA291C",20,40,"F8E600",40,60,"007A33"]}}
    //perú {"seg":{"i":[0,20,"FF0000",20,25,"FFFFFF",25,27,"1ccc00",27,28,"FFFFFF",28,32,"FF0000",32,33,"FFFFFF",33,35,"1ccc00",35,40,"FFFFFF",40,60,"FF0000"]}}
    //     {"seg":{"i":[0,20,"FF0000",20,40,"FFFFFF",40,60,"FF0000"]}}
    //Ecuador 

  }

  public String gsonSerialize(LedSegment segment){
    if(segment.getData().isUpdated()){
      GsonBuilder builder = new GsonBuilder();
      builder.registerTypeAdapter(frc.robot.Subsystems.Lighting.WLED.LedSegment.LedMultiRange.class, new LedMultiRangeSerializer());
      builder.registerTypeAdapter(Optional.class, new OptionalSerializer());
      builder.registerTypeAdapter(CustomColor.class, new ColorSerializer());
      Gson gson = builder.create();
      String json = gson.toJson(segment.getData());
      return "{\"seg\":"+json +"}";
    }
    else{
      return "";
    }
    
  }

  private class LedRangeSerializer implements JsonSerializer<frc.robot.Subsystems.Lighting.WLED.LedSegment.LedRange>{
    public JsonElement serialize(frc.robot.Subsystems.Lighting.WLED.LedSegment.LedRange ledRange, Type type, JsonSerializationContext jsonSerializationContext){
      JsonArray array = new JsonArray();
      array.add(ledRange.getStart());
      array.add(ledRange.getStop());
      array.add(ledRange.getColor().getHex());
      return array;
    }
  }



  public class LedMultiRangeSerializer implements JsonSerializer<frc.robot.Subsystems.Lighting.WLED.LedSegment.LedMultiRange>{
    public JsonElement serialize(frc.robot.Subsystems.Lighting.WLED.LedSegment.LedMultiRange ledRange, Type type, JsonSerializationContext jsonSerializationContext){
      JsonArray array = new JsonArray();
      for (int i = 0; i<ledRange.getLedRanges().size();i++){
        array.add(ledRange.getLedRanges().get(i).getStart());
        array.add(ledRange.getLedRanges().get(i).getStop());
        array.add(ledRange.getLedRanges().get(i).getColor().getHex());
      }
      return array;
    }
  }

  private class OptionalSerializer implements JsonSerializer<Optional<?>> {
    public JsonElement serialize(Optional<?> optional, Type type, JsonSerializationContext jsonSerializationContext){
      return jsonSerializationContext.serialize(optional.orElse(null));
    }
  }

  private class ColorSerializer implements JsonSerializer<CustomColor> {
    public JsonElement serialize(CustomColor color, Type type, JsonSerializationContext jsonSerializationContext){
      return jsonSerializationContext.serialize(color.getHex());
    }
  }
}


