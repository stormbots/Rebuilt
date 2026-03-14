// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Lighting;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/** Add your docs here. */
public class LedSegment extends LedBase {
    public LedSegmentData data;
    private int length;
    public LedSegment(int id, int start, int stop, boolean reverse){
      super(id, start, stop, reverse);
      data = new LedSegmentData(id, start, stop, reverse);
      this.length = stop - start;
      WLED.registerSegment(this);
    }

    /**Added because having issues with .equals, left because it works */
    public boolean compareColors(CustomColor[] col1, CustomColor[] col2){
      if (col1.length != col2.length){
        return false;
      }
      for (int i = 0; i<col1.length;i++){
        if(!col1[i].equals(col2[i])){
          return false;
        }
      }
      return true;

    }

    public void setColor(CustomColor... color){
      boolean colorReset =false;
      if(this.col == null || !compareColors(col, color)){
        if(this.col != null){
        if (color.length<col.length){
          for (int i = 0; i < col.length;i++){
            if (i<color.length){
              col[i] = color[i];
            }
            else if (!col[i].equals(CustomColor.kBlack)){ 
              col[i] = CustomColor.kBlack;
              colorReset = true;
            }
          }
          if (colorReset){
            color = this.col;
            }
            else{
              this.col = color;
            }   
          }
          else{
            this.col = color;
          }   
        }
        else{
          this.col = color;
        }
       
        data.setColor(color);
        setFreeze(false); //if individual control is set, it freezes the segment, this is called to automatically acount for that
        setPalette(0);
      }
    }

    public void setEffect(int effect){
      if(this.fx.isEmpty() || !this.fx.get().equals(effect)){
        this.fx = Optional.of(effect);
        data.setEffect(effect);
        setFreeze(false);
      }
    }

    public void setSpeed(int speed){
      if(this.sx.isEmpty() || !this.sx.get().equals(speed)){
        this.sx = Optional.of(speed);
        data.setSpeed(speed);
        setFreeze(false);
      }
    }

    public void setIntensity(int intensity){
      if(this.ix.isEmpty() || !this.ix.get().equals(intensity)){
        this.ix = Optional.of(intensity);
        data.setIntensity(intensity);
        setFreeze(false);
      }
    }
    
    public void setCustomSlider1(int custom1){
      if(this.c1.isEmpty() || !this.c1.get().equals(custom1)){
        this.c1 = Optional.of(custom1);
        data.setCustomSlider1(custom1);
        setFreeze(false);
      }
    }

    public void setCustomSlider2(int custom2){
      if(this.c2.isEmpty() || !this.c2.get().equals(custom2)){
        this.c2 = Optional.of(custom2);
        data.setCustomSlider2(custom2);
        setFreeze(false);
      }
    }

    public void setCustomSlider3(int custom3){
      if(this.c3.isEmpty() || !this.c3.get().equals(custom3)){
        this.c3 = Optional.of(custom3);
        data.setCustomSlider3(custom3);
        setFreeze(false);
      }
    }

    public void setReverse(boolean reverse){
      if(this.rev.isEmpty() || !this.rev.get().equals(reverse)){
        this.rev = Optional.of(reverse);
        data.setReverse(reverse);
        setFreeze(false);
      }
    }

    public void setOn(boolean setOn){
      if(this.on.isEmpty() || !this.on.get().equals(setOn)){
        this.on = Optional.of(setOn);
        data.setOn(setOn);
        setFreeze(false);
      }
    }

    public void setBrightness(int brightness){
      if(this.bri.isEmpty() || !this.bri.get().equals(brightness)){
        this.bri = Optional.of(brightness);
        data.setBrightness(brightness);
        setFreeze(false);
      }
    }

    public void setPalette(int palette){
      if(this.pal.isEmpty() || !this.pal.get().equals(palette)){
        this.pal = Optional.of(palette);
        data.setPalette(palette);
        setFreeze(false);
      }
    }

    public void setIndividualControl(LedMultiRange ind){
      if(this.i == null||!this.i.equals(ind)){
        this.i = ind;
        data.setIndividualControl(ind);
        frz = Optional.of(true);
      }
    }

    public void setFreeze(boolean freeze){
      if(this.frz.isEmpty() || !this.frz.get().equals(freeze)){
        this.frz = Optional.of(freeze);
        data.setFreeze(freeze);
      }
    }

    public void reset(){
      data.reset();
    }

    public LedSegmentData getData(){
      return data;
    }

    public Command solidColor(CustomColor color){
      return new InstantCommand(()->{
        setColor(color);
        setEffect(0);
      }, this);
    }

    public Command blink(CustomColor color, int speed){
      return new InstantCommand(()->{
        setEffect(1);
        setColor(color);
        setSpeed(speed);
      }, this);
    }

    public Command blinkSmooth(int speed, CustomColor...color){
      return new InstantCommand(()->{
        setEffect(108);
        setColor(color);
        setSpeed(speed);
        setIntensity(0);
      }, this);
    }

  

    // TODO: Add compensation for unset start/stop
    public class LedRange{
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

      public LedRange(CustomColor color, double fraction, double index){
        int pixels = (int) Math.round(length/fraction);
        if (pixels == 0){
          pixels = 1;
        }
        start = (int) Math.round(pixels*(index-1));
        stop = (int)Math.round(pixels*index);
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
    public class LedMultiRange{
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
          int fraction = Math.round(length/list.length);
          if (fraction == 0){
            fraction = 1;
          }

          for(int i=0; i<list.length; i++){
            if (i>0 && list[i].getColor() == list[i-1].getColor()){
              instances++;
              int newFraction = Math.round(instances*length/list.length);
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

    public Command noAprilTags(){
      return new InstantCommand(()->{
        setEffect(110);
        setIntensity(1);
      }, this);
    }

    public Command showAllianceColorInteresting(){
      return Commands.select(getPatternMap(), ()->WLED.getDefaultPattern());
    }

    private CustomColor getAllianceColor(){
      if (DriverStation.getAlliance().isPresent()){
        if (DriverStation.getAlliance().get().equals(Alliance.Red)){
          return CustomColor.kRed;
        }
        else return CustomColor.kBlue;
      }
      else return CustomColor.kPurple;
    }
      

    private int getAlliancePalatte(){
      if (DriverStation.getAlliance().isPresent()){
        if (DriverStation.getAlliance().get().equals(Alliance.Red)){
          return 35;
        }
        else return 36;
      }
      else return 0;
    }
    private HashMap<Integer,Command> getPatternMap(){
      
      Supplier<CustomColor> colorSupplier = ()-> getAllianceColor();

      
      Supplier<Integer> palatteSupplier = ()-> getAlliancePalatte();

      HashMap<Integer,Command> patternMap = new HashMap<>();
      patternMap.put(0, Commands.runOnce(()->{
        setEffect(27);
        setColor(colorSupplier.get());
        setSpeed(128);
        setIntensity(128);
      }, this));

      patternMap.put(1, Commands.runOnce(()->{
        setEffect(68);
        setColor(colorSupplier.get());
        setSpeed(64);
      }, this));

      patternMap.put(2, Commands.runOnce(()->{
        setEffect(2);
        setColor(colorSupplier.get());
        setSpeed(128);
      }, this));

      patternMap.put(3, Commands.runOnce(()->{
        setEffect(102);
        setColor(colorSupplier.get());
        setSpeed(200);
        setIntensity(255);
      }, this));

      patternMap.put(4, Commands.runOnce(()->{
        setEffect(28);
        setColor(colorSupplier.get());
        setSpeed(128);
        setIntensity(128);
      }, this));

      patternMap.put(5, Commands.runOnce(()->{
        setEffect(111);
        setColor(colorSupplier.get());
        setSpeed(25);
        setIntensity(128);
      }, this));

      patternMap.put(6, Commands.runOnce(()->{
        setEffect(67);
        setColor(colorSupplier.get());
        setSpeed(64);
        setIntensity(128);
      }, this));

      patternMap.put(7, Commands.runOnce(()->{
        setEffect(12);
        setColor(colorSupplier.get());
        setSpeed(25);
      }, this));

      patternMap.put(8, Commands.runOnce(()->{
        setEffect(66);
        setPalette(palatteSupplier.get());
        setSpeed(64);
        setIntensity(160);
        setCustomSlider1(16);
      }, this));

      patternMap.put(9, Commands.runOnce(()->{
        setEffect(46);
        setColor(colorSupplier.get());
        setSpeed(128);
        setIntensity(62);
      }, this));

      patternMap.put(10, Commands.runOnce(()->{
        setEffect(132);
        setColor(colorSupplier.get());
        setSpeed(128);
        setIntensity(128);
      }, this));

      patternMap.put(11, Commands.runOnce(()->{
        setEffect(156);
        setColor(colorSupplier.get());
        setSpeed(128);
        setIntensity(128);
      }, this));

      patternMap.put(12, Commands.runOnce(()->{
        setEffect(41);
        setColor(colorSupplier.get());
        setSpeed(128);
        setIntensity(128);
      }, this));

      patternMap.put(13, Commands.runOnce(()->{
        setEffect(47);
        setColor(colorSupplier.get());
        setSpeed(128);
        setIntensity(16);
      }, this));

      patternMap.put(14, Commands.runOnce(()->{
        setEffect(131);
        setColor(colorSupplier.get());
        setSpeed(128);
        setIntensity(64);
      }, this));

      patternMap.put(15, Commands.runOnce(()->{
        setEffect(76);
        setPalette(palatteSupplier.get());
        setSpeed(128);
        setIntensity(128);
      }, this));

      patternMap.put(16, Commands.runOnce(()->{
        setEffect(135);
        setColor(colorSupplier.get());
        setSpeed(128);
        setIntensity(128);
      }, this));

      patternMap.put(17, Commands.runOnce(()->{
        setEffect(133);
        setColor(colorSupplier.get());
        setSpeed(128);
        setIntensity(128);
      }, this));

      patternMap.put(18, Commands.runOnce(()->{
        setEffect(15);
        setColor(colorSupplier.get());
        setSpeed(128);
        setIntensity(128);
      }, this));

      patternMap.put(19, Commands.runOnce(()->{
        setEffect(16);
        setColor(colorSupplier.get());
        setSpeed(128);
      }, this));

      patternMap.put(20, Commands.runOnce(()->{
        setEffect(108);
        setColor(colorSupplier.get());
        setSpeed(64);
        setIntensity(32);
      }, this));

      patternMap.put(21, Commands.runOnce(()->{
        setEffect(113);
        setColor(colorSupplier.get());
        setPalette(4);
        setSpeed(128);
        setSpeed(128);
      }, this));

      patternMap.put(22, Commands.runOnce(()->{
        setEffect(108);
        setColor(colorSupplier.get());
        setSpeed(32);
        setIntensity(0);
      }, this));

      patternMap.put(WLED.numPatterns, pride());

      patternMap.put(WLED.numPatterns+1, seguimosAqui());

      patternMap.put(WLED.numPatterns+2, solidColor(CustomColor.kBlack));

      patternMap.put(WLED.numPatterns+3, solidColor(colorSupplier.get()));

      return patternMap;

    }

    public Command debugStrips(){
      SmartDashboard.putNumber("leds/Strip1Start", 1);
      SmartDashboard.putNumber("leds/Strip1end", 16);
      SmartDashboard.putNumber("leds/Strip2Start", 18);
      SmartDashboard.putNumber("leds/Strip2end", 26);
      SmartDashboard.putNumber("leds/Strip3Start", 28);
      SmartDashboard.putNumber("leds/Strip3end", 36);

      Supplier<Integer> start1 = ()->(int)SmartDashboard.getNumber("leds/Strip1Start", 1);
      Supplier<Integer> stop1 = ()->(int)SmartDashboard.getNumber("leds/Strip1end", 16);
      Supplier<Integer> start2 = ()->(int)SmartDashboard.getNumber("leds/Strip2Start", 18);
      Supplier<Integer> stop2 = ()->(int)SmartDashboard.getNumber("leds/Strip2end", 26);
      Supplier<Integer> start3 = ()->(int)SmartDashboard.getNumber("leds/Strip3Start", 28);
      Supplier<Integer> stop3 = ()->(int)SmartDashboard.getNumber("leds/Strip3end", 36);
      LedMultiRange debug = new LedMultiRange(new LedRange(CustomColor.kBlack), 
      new LedRange(start1.get(), stop1.get(), CustomColor.kRed),
      new LedRange(CustomColor.kBlack),
      new LedRange(start2.get(), stop2.get(), CustomColor.kGreen),
      new LedRange(CustomColor.kBlack),
      new LedRange(start3.get(), stop3.get(), CustomColor.kBlue),
      new LedRange(CustomColor.kBlack));

      return stripes(debug);
    }

    

    public Command pride(){
      double duration = 2.5;
      LedMultiRange trans = new LedMultiRange(true, new LedRange(CustomColor.kBabyBlue),new LedRange(CustomColor.kPink), new LedRange(CustomColor.kWhite),new LedRange(CustomColor.kPink),new LedRange(CustomColor.kBabyBlue));
      LedMultiRange pride = new LedMultiRange(true, new LedRange(CustomColor.kRed),new LedRange(CustomColor.kOrange), new LedRange(CustomColor.kYellow),new LedRange(CustomColor.kGreen),new LedRange(CustomColor.kBlue),new LedRange(CustomColor.kPurple));
      LedMultiRange enby = new LedMultiRange(true, new LedRange(CustomColor.kYellow),new LedRange(CustomColor.kWhite), new LedRange(CustomColor.kPurple),new LedRange(CustomColor.kBlack));
      LedMultiRange bi = new LedMultiRange(new LedRange(CustomColor.kMagenta),new LedRange((int)(length/5.0*2),(int)(length/5.0*3),CustomColor.kPurple), new LedRange(CustomColor.kBlue));
      LedMultiRange intersex = new LedMultiRange(new LedRange(CustomColor.kYellow),new LedRange((int)(length/18.0*6),(int)(length/18.0*7),CustomColor.kPurple), new LedRange(CustomColor.kYellow),new LedRange((int)(length/18.0*11),(int)(length/18.0*12),CustomColor.kPurple),new LedRange(CustomColor.kYellow));
      LedMultiRange ace = new LedMultiRange(true, new LedRange(CustomColor.kBlack),new LedRange(CustomColor.kGrey), new LedRange(CustomColor.kWhite),new LedRange(CustomColor.kPurple));
      LedMultiRange aro = new LedMultiRange(true, new LedRange(CustomColor.kGreen),new LedRange(CustomColor.kLightGreen), new LedRange(CustomColor.kWhite),new LedRange(CustomColor.kGrey),new LedRange(CustomColor.kBlack));

      return new SequentialCommandGroup(stripes(trans)
      .andThen(new WaitCommand(duration))
      .andThen(stripes(pride))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(enby))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(bi))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(intersex))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(ace))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(aro))
      .andThen(new WaitCommand(duration)));
    }

    /** "God bless America. Sea Chile, Argentina, Uruguay, Paraguay, Bolivia, Perú, Ecuador, Brasil, Colombia, Venezuela, Guyana, Panamá, Costa Rica, Nicaragua, Honduras, El Salvador, Guatemala, México, Cuba, República Dominicana, Jamaica, Haití, Las Antillas, United States, Canadá, and my motherland, mi patria, Puerto Rico. Seguimos aquí"
     * <p>
     * - Benito Antonio Martínez Ocasio, 2026
    */
    public Command seguimosAqui(){
      double duration = 2.5;
      LedMultiRange chile = new LedMultiRange(new LedRange(CustomColor.kChileBlue),new LedRange(CustomColor.kWhite,15,3), new LedRange(CustomColor.kChileBlue),new LedRange(CustomColor.kWhite,3,2),new LedRange(CustomColor.kRed));
      LedMultiRange argentina = new LedMultiRange(new LedRange(CustomColor.kArgentinaBlue),new LedRange(CustomColor.kWhite,12,5), new LedRange(CustomColor.kArgentinaYellow),new LedRange(CustomColor.kWhite,12,8),new LedRange(CustomColor.kArgentinaBlue));
      LedMultiRange uruguay = new LedMultiRange(new LedRange(CustomColor.kWhite),new LedRange(CustomColor.kUruguayYellow,3.75,23.0/16), new LedRange(CustomColor.kWhite),new LedRange(CustomColor.kBlue,9,6),new LedRange(CustomColor.kWhite),new LedRange(CustomColor.kBlue,9,8),new LedRange(CustomColor.kWhite));
      LedMultiRange paraguay = new LedMultiRange(new LedRange(CustomColor.kRed),new LedRange(CustomColor.kWhite,12,5), new LedRange(CustomColor.kParaguayGreen),new LedRange(CustomColor.kWhite,60,28),new LedRange(CustomColor.kUruguayYellow),new LedRange(CustomColor.kWhite,60,33),new LedRange(CustomColor.kParaguayGreen),new LedRange(CustomColor.kWhite,12,8),new LedRange(CustomColor.kRed));
      LedMultiRange bolivia = new LedMultiRange(true, new LedRange(CustomColor.kBoliviaRed),new LedRange(CustomColor.kBoliviaYellow), new LedRange(CustomColor.kBoliviaGreen));
      LedMultiRange peru = new LedMultiRange(true, new LedRange(CustomColor.kRed),new LedRange(CustomColor.kWhite), new LedRange(CustomColor.kRed));
      LedMultiRange ecuador = new LedMultiRange(new LedRange(CustomColor.kEcuadorYellow),new LedRange(CustomColor.kEcuadorGreen,7.5,4.25), new LedRange(CustomColor.kEcuadorBlue),new LedRange(CustomColor.kRed,4,4));
      LedMultiRange brasil = new LedMultiRange(new LedRange(CustomColor.kBrazilGreen,5,1),new LedRange(CustomColor.kBrazilYellow), new LedRange(CustomColor.kBrazilBlue,60.0/14,37.0/14),new LedRange(CustomColor.kBrazilYellow),new LedRange(CustomColor.kBrazilGreen,5,5));
      LedMultiRange colombia = new LedMultiRange(new LedRange(CustomColor.kEcuadorYellow),new LedRange(CustomColor.kEcuadorBlue,4,3), new LedRange(CustomColor.kRed));
      LedMultiRange venezuela = new LedMultiRange(new LedRange(CustomColor.kVenezuelaYellow,3,1),new LedRange(CustomColor.kVenezuelaBlue), new LedRange(CustomColor.kWhite,30,13),new LedRange(CustomColor.kVenezuelaBlue),new LedRange(CustomColor.kWhite,30,15.5),new LedRange(CustomColor.kVenezuelaBlue),new LedRange(CustomColor.kWhite,30,18),new LedRange(CustomColor.kVenezuelaBlue),new LedRange(CustomColor.kRed,3,3));
      LedMultiRange guyana = new LedMultiRange(new LedRange(CustomColor.kGuyanaRed),new LedRange(CustomColor.kBlack,30,10), new LedRange(CustomColor.kEcuadorYellow), new LedRange(CustomColor.kWhite,30,17.5),new LedRange(CustomColor.kGuyanaGreen));
      LedMultiRange panama = new LedMultiRange(new LedRange(CustomColor.kWhite),new LedRange(CustomColor.kPanamaBlue,20,3), new LedRange(CustomColor.kWhite),new LedRange(CustomColor.kPanamaBlue,4,2),new LedRange(CustomColor.kWhite),new LedRange(CustomColor.kPanamaRed,20,13),new LedRange(CustomColor.kWhite),new LedRange(CustomColor.kPanamaRed,4,4));
      LedMultiRange costaRica = new LedMultiRange(new LedRange(CustomColor.kCostaRicaBlue),new LedRange(CustomColor.kWhite,10,2), new LedRange(CustomColor.kGuyanaRed),new LedRange(CustomColor.kWhite,10,6),new LedRange(CustomColor.kCostaRicaBlue));
      LedMultiRange nicaragua = new LedMultiRange(new LedRange(CustomColor.kNicaraguaBlue),new LedRange(CustomColor.kWhite,7.5,3.5), new LedRange(CustomColor.kNicaraguaGreen),new LedRange(CustomColor.kWhite,7.5,5),new LedRange(CustomColor.kNicaraguaBlue));
      LedMultiRange honduras = new LedMultiRange(new LedRange(CustomColor.kHondurasBlue),new LedRange(CustomColor.kWhite,15,6), new LedRange(CustomColor.kHondurasBlue),new LedRange(CustomColor.kWhite,20,29.0/3),new LedRange(CustomColor.kHondurasBlue),new LedRange(CustomColor.kWhite,20,34.0/3),new LedRange(CustomColor.kHondurasBlue),new LedRange(CustomColor.kWhite,15,10),new LedRange(CustomColor.kHondurasBlue));
      LedMultiRange elSalvador = new LedMultiRange(new LedRange(CustomColor.kElSalvadorBlue),new LedRange(CustomColor.kWhite,10,26.0/6), new LedRange(CustomColor.kElSalvadorGreen),new LedRange(CustomColor.kWhite,10,40.0/6),new LedRange(CustomColor.kElSalvadorBlue));
      LedMultiRange guatemala = new LedMultiRange(new LedRange(CustomColor.kGuatemalaBlue),new LedRange(CustomColor.kWhite,12,5), new LedRange(CustomColor.kGuatemalaGreen),new LedRange(CustomColor.kWhite,12,8),new LedRange(CustomColor.kGuatemalaBlue));
      LedMultiRange mexico = new LedMultiRange(new LedRange(CustomColor.kMexicoGreen),new LedRange(CustomColor.kWhite,60.0/7,27.0/7), new LedRange(CustomColor.kMexicoBrown),new LedRange(CustomColor.kWhite,60.0/7,40.0/7), new LedRange(CustomColor.kMexicoRed));
      LedMultiRange cuba = new LedMultiRange(new LedRange(CustomColor.kCubaBlue),new LedRange(CustomColor.kWhite,7.5,2), new LedRange(CustomColor.kCubaBlue),new LedRange(CustomColor.kBoliviaRed,10,29.0/6),new LedRange(CustomColor.kWhite),new LedRange(CustomColor.kBoliviaRed,10,37.0/6), new LedRange(CustomColor.kCubaBlue),new LedRange(CustomColor.kWhite,7.5,52.0/8) ,new LedRange(CustomColor.kCubaBlue));
      LedMultiRange repeblicaDominicana = new LedMultiRange(new LedRange(CustomColor.kRepublicaDominicanaBlue),new LedRange(CustomColor.kWhite,15,4), new LedRange(CustomColor.kRepublicaDominicanaRed),new LedRange(CustomColor.kRepublicaDominicanaGreen,15,8), new LedRange(CustomColor.kRepublicaDominicanaBlue), new LedRange(CustomColor.kWhite,15,12),new LedRange(CustomColor.kRepublicaDominicanaRed));
      LedMultiRange jamaica = new LedMultiRange(true, new LedRange(CustomColor.kJamaicaGreen),new LedRange(CustomColor.kEcuadorYellow), new LedRange(CustomColor.kBlack),new LedRange(CustomColor.kEcuadorYellow),new LedRange(CustomColor.kJamaicaGreen));
      LedMultiRange haiti = new LedMultiRange(new LedRange(CustomColor.kHaitiBlue),new LedRange(CustomColor.kWhite,7.5,4.25), new LedRange(CustomColor.kHaitiRed));
      LedMultiRange unitedStates = new LedMultiRange(new LedRange(CustomColor.kUnitedStatesBlue),new LedRange(CustomColor.kWhite,30,4), new LedRange(CustomColor.kUnitedStatesBlue),new LedRange(CustomColor.kWhite,30,8),new LedRange(CustomColor.kUnitedStatesBlue),new LedRange(CustomColor.kWhite,30,12),new LedRange(CustomColor.kUnitedStatesBlue),new LedRange(CustomColor.kWhite,12,7),new LedRange(CustomColor.kUnitedStatesRed),new LedRange(CustomColor.kWhite,12,9),new LedRange(CustomColor.kUnitedStatesRed),new LedRange(CustomColor.kWhite,12,11),new LedRange(CustomColor.kUnitedStatesRed));
      LedMultiRange canada = new LedMultiRange(new LedRange(CustomColor.kCanadaRed),new LedRange(CustomColor.kWhite,12,5), new LedRange(CustomColor.kCanadaRed),new LedRange(CustomColor.kWhite,12,8),new LedRange(CustomColor.kCanadaRed));
      LedMultiRange puertoRico = new LedMultiRange(new LedRange(CustomColor.kPuertoRicoRed),new LedRange(CustomColor.kWhite,7.5,2), new LedRange(CustomColor.kPuertoRicoRed),new LedRange(CustomColor.kPuertoRicoBlue,10,29.0/6),new LedRange(CustomColor.kWhite),new LedRange(CustomColor.kPuertoRicoBlue,10,37.0/6), new LedRange(CustomColor.kPuertoRicoRed),new LedRange(CustomColor.kWhite,7.5,52.0/8) ,new LedRange(CustomColor.kPuertoRicoRed));
      
      return new SequentialCommandGroup(stripes(chile)
      .andThen(new WaitCommand(duration))
      .andThen(stripes(argentina))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(uruguay))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(paraguay))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(bolivia))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(peru))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(ecuador))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(brasil))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(colombia))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(venezuela))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(guyana))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(panama))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(costaRica))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(nicaragua))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(honduras))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(elSalvador))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(guatemala))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(mexico))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(cuba))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(repeblicaDominicana))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(jamaica))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(haiti))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(unitedStates))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(canada))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(puertoRico))
      .andThen(new WaitCommand(duration)));
    }
  }
