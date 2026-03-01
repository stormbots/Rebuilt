// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Lighting;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
      if(this.col == null || !compareColors(col, color)){
        this.col = color;
        data.setColor(color);
        setFreeze(false); //if individual control is set, it freezez the segment, this is called to authomattically acount for that
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
      if(this.pal.isEmpty() || !this.bri.get().equals(palette)){
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
        setColor(color,CustomColor.kBlack);
        setSpeed(speed);
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
    public Command seguimosAquí(){
      double duration = 2.5;
      LedMultiRange chile = new LedMultiRange(new LedRange(CustomColor.kChileBlue),new LedRange(CustomColor.kWhite,15,3), new LedRange(CustomColor.kChileBlue),new LedRange(CustomColor.kWhite,3,2),new LedRange(CustomColor.kRed));
      LedMultiRange argentina = new LedMultiRange(new LedRange(CustomColor.kArgentinaBlue),new LedRange(CustomColor.kWhite,12,5), new LedRange(CustomColor.kArgentinaYellow),new LedRange(CustomColor.kWhite,12,8),new LedRange(CustomColor.kArgentinaBlue));
      LedMultiRange uruguay = new LedMultiRange(new LedRange(CustomColor.kWhite),new LedRange(CustomColor.kUruguayYellow,3.75,23.0/16), new LedRange(CustomColor.kWhite),new LedRange(CustomColor.kBlue,9,6),new LedRange(CustomColor.kWhite),new LedRange(CustomColor.kBlue,9,8),new LedRange(CustomColor.kWhite));
      LedMultiRange paraguay = new LedMultiRange(new LedRange(CustomColor.kRed),new LedRange(CustomColor.kWhite,12,5), new LedRange(CustomColor.kParaguayGreen),new LedRange(CustomColor.kWhite,60,28),new LedRange(CustomColor.kUruguayYellow),new LedRange(CustomColor.kWhite,60,33),new LedRange(CustomColor.kParaguayGreen),new LedRange(CustomColor.kWhite,12,8),new LedRange(CustomColor.kRed));
      LedMultiRange bolivia = new LedMultiRange(true, new LedRange(CustomColor.kBoliviaRed),new LedRange(CustomColor.kBoliviaYellow), new LedRange(CustomColor.kBoliviaGreen));
      LedMultiRange perú = new LedMultiRange(true, new LedRange(CustomColor.kRed),new LedRange(CustomColor.kWhite), new LedRange(CustomColor.kRed));
      LedMultiRange ecuador = new LedMultiRange(new LedRange(CustomColor.kEcuadorYellow),new LedRange(CustomColor.kEcuadorGreen,7.5,4.25), new LedRange(CustomColor.kEcuadorBlue),new LedRange(CustomColor.kRed,4,4));
      LedMultiRange brasil = new LedMultiRange(new LedRange(CustomColor.kBrazilGreen,5,1),new LedRange(CustomColor.kBrazilYellow), new LedRange(CustomColor.kBrazilBlue,60.0/14,37.0/14),new LedRange(CustomColor.kBrazilYellow),new LedRange(CustomColor.kBrazilGreen,5,5));
      LedMultiRange colombia = new LedMultiRange(new LedRange(CustomColor.kEcuadorYellow),new LedRange(CustomColor.kEcuadorBlue,4,3), new LedRange(CustomColor.kRed));
      LedMultiRange venezuela = new LedMultiRange(new LedRange(CustomColor.kVenezuelaYellow,3,1),new LedRange(CustomColor.kVenezuelaBlue), new LedRange(CustomColor.kWhite,30,13),new LedRange(CustomColor.kVenezuelaBlue),new LedRange(CustomColor.kWhite,30,15.5),new LedRange(CustomColor.kVenezuelaBlue),new LedRange(CustomColor.kWhite,30,18),new LedRange(CustomColor.kVenezuelaBlue),new LedRange(CustomColor.kRed,3,3));
      LedMultiRange guyana = new LedMultiRange(new LedRange(CustomColor.kGuyanaRed),new LedRange(CustomColor.kBlack,30,10), new LedRange(CustomColor.kEcuadorYellow), new LedRange(CustomColor.kWhite,30,17.5),new LedRange(CustomColor.kGuyanaGreen));
      LedMultiRange panamá = new LedMultiRange(new LedRange(CustomColor.kWhite),new LedRange(CustomColor.kPanamáBlue,20,3), new LedRange(CustomColor.kWhite),new LedRange(CustomColor.kPanamáBlue,4,2),new LedRange(CustomColor.kWhite),new LedRange(CustomColor.kPanamáRed,20,13),new LedRange(CustomColor.kWhite),new LedRange(CustomColor.kPanamáRed,4,4));
      LedMultiRange costRica = new LedMultiRange(new LedRange(CustomColor.kCostaRicaBlue),new LedRange(CustomColor.kWhite,10,2), new LedRange(CustomColor.kGuyanaRed),new LedRange(CustomColor.kWhite,10,6),new LedRange(CustomColor.kCostaRicaBlue));
      LedMultiRange nicaragua = new LedMultiRange(new LedRange(CustomColor.kNicaraguaBlue),new LedRange(CustomColor.kWhite,7.5,3.5), new LedRange(CustomColor.kNicaraguaGreen),new LedRange(CustomColor.kWhite,7.5,5),new LedRange(CustomColor.kNicaraguaBlue));
      LedMultiRange honduras = new LedMultiRange(new LedRange(CustomColor.kHondurasBlue),new LedRange(CustomColor.kWhite,15,6), new LedRange(CustomColor.kHondurasBlue),new LedRange(CustomColor.kWhite,20,29.0/3),new LedRange(CustomColor.kHondurasBlue),new LedRange(CustomColor.kWhite,20,34.0/3),new LedRange(CustomColor.kHondurasBlue),new LedRange(CustomColor.kWhite,15,10),new LedRange(CustomColor.kHondurasBlue));
      LedMultiRange elSalvador = new LedMultiRange(new LedRange(CustomColor.kElSalvadorBlue),new LedRange(CustomColor.kWhite,10,26.0/6), new LedRange(CustomColor.kElSalvadorGreen),new LedRange(CustomColor.kWhite,10,40.0/6),new LedRange(CustomColor.kElSalvadorBlue));
      LedMultiRange guatemala = new LedMultiRange(new LedRange(CustomColor.kGuatemalaBlue),new LedRange(CustomColor.kWhite,12,5), new LedRange(CustomColor.kGuatemalaGreen),new LedRange(CustomColor.kWhite,12,8),new LedRange(CustomColor.kGuatemalaBlue));
      LedMultiRange méxico = new LedMultiRange(new LedRange(CustomColor.kMéxicoGreen),new LedRange(CustomColor.kWhite,60.0/7,27.0/7), new LedRange(CustomColor.kMéxicoBrown),new LedRange(CustomColor.kWhite,60.0/7,40.0/7), new LedRange(CustomColor.kMéxicoRed));
      LedMultiRange cuba = new LedMultiRange(new LedRange(CustomColor.kCubaBlue),new LedRange(CustomColor.kWhite,7.5,2), new LedRange(CustomColor.kCubaBlue),new LedRange(CustomColor.kBoliviaRed,10,29.0/6),new LedRange(CustomColor.kWhite),new LedRange(CustomColor.kBoliviaRed,10,37.0/6), new LedRange(CustomColor.kCubaBlue),new LedRange(CustomColor.kWhite,7.5,52.0/8) ,new LedRange(CustomColor.kCubaBlue));
      LedMultiRange repúblicaDominicana = new LedMultiRange(new LedRange(CustomColor.kRepúblicaDominicanaBlue),new LedRange(CustomColor.kWhite,15,4), new LedRange(CustomColor.kRepúblicaDominicanaRed),new LedRange(CustomColor.kRepúblicaDominicanaGreen,15,8), new LedRange(CustomColor.kRepúblicaDominicanaBlue), new LedRange(CustomColor.kWhite,15,12),new LedRange(CustomColor.kRepúblicaDominicanaRed));
      LedMultiRange jamaica = new LedMultiRange(true, new LedRange(CustomColor.kJamaicaGreen),new LedRange(CustomColor.kEcuadorYellow), new LedRange(CustomColor.kBlack),new LedRange(CustomColor.kEcuadorYellow),new LedRange(CustomColor.kJamaicaGreen));
      LedMultiRange haití = new LedMultiRange(new LedRange(CustomColor.kHaitíBlue),new LedRange(CustomColor.kWhite,7.5,4.25), new LedRange(CustomColor.kHaitíRed));
      LedMultiRange unitedStates = new LedMultiRange(new LedRange(CustomColor.kUnitedStatesBlue),new LedRange(CustomColor.kWhite,30,4), new LedRange(CustomColor.kUnitedStatesBlue),new LedRange(CustomColor.kWhite,30,8),new LedRange(CustomColor.kUnitedStatesBlue),new LedRange(CustomColor.kWhite,30,12),new LedRange(CustomColor.kUnitedStatesBlue),new LedRange(CustomColor.kWhite,12,7),new LedRange(CustomColor.kUnitedStatesRed),new LedRange(CustomColor.kWhite,12,9),new LedRange(CustomColor.kUnitedStatesRed),new LedRange(CustomColor.kWhite,12,11),new LedRange(CustomColor.kUnitedStatesRed));
      LedMultiRange canadá = new LedMultiRange(new LedRange(CustomColor.kCanadáRed),new LedRange(CustomColor.kWhite,12,5), new LedRange(CustomColor.kCanadáRed),new LedRange(CustomColor.kWhite,12,8),new LedRange(CustomColor.kCanadáRed));
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
      .andThen(stripes(perú))
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
      .andThen(stripes(panamá))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(costRica))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(nicaragua))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(honduras))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(elSalvador))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(guatemala))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(méxico))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(cuba))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(repúblicaDominicana))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(jamaica))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(haití))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(unitedStates))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(canadá))
      .andThen(new WaitCommand(duration))
      .andThen(stripes(puertoRico))
      .andThen(new WaitCommand(duration)));
    }



    // seguimos aquí
    // Chile {"seg":{"i":[0,8,"0032A0",8,12,"FFFFFF",12,20,"0032A0",20,40,"FFFFFF",40,60,"DA291C"]}}
    // Argentina {"seg":{"i":[0,20,"6CACE4",20,25,"FFFFFF",25,35,"FFB81C",35,40,"FFFFFF",40,60,"6CACE4"]}}
    // Uruguay {"seg":{"i":[0,7,"FFFFFF",7,23,"FFCD00",23,30,"FFFFFF",30,37,"0000FF",37,44,"FFFFFF",44,51,"0000FF",51,60,"FFFFFF"]}} 9 stripes
    // Paraguay {"seg":{"i":[0,20,"FF0000",20,25,"FFFFFF",25,27,"1ccc00",27,28,"FFFFFF",28,32,"FFCD00",32,33,"FFFFFF",33,35,"1ccc00",35,40,"FFFFFF",40,60,"0000FF"]}}
    // Bolivia {"seg":{"i":[0,20,"DA291C",20,26,"F8E600",26,29,"DA291C",29,31,"F8E600",31,34,"DA291C",34,40,"F8E600",40,60,"007A33"]}}
    //        {"seg":{"i":[0,20,"DA291C",20,40,"F8E600",40,60,"007A33"]}}
    // Perú {"seg":{"i":[0,20,"FF0000",20,25,"FFFFFF",25,27,"1ccc00",27,28,"FFFFFF",28,32,"FF0000",32,33,"FFFFFF",33,35,"1ccc00",35,40,"FFFFFF",40,60,"FF0000"]}}
    //     {"seg":{"i":[0,20,"FF0000",20,40,"FFFFFF",40,60,"FF0000"]}}
    // Ecuador {"seg":{"i":[0,26,"FFD100",26,34,"0e9c1f",34,45,"0072CE",45,60,"FF0000"]}}
    // Brasil {"seg":{"i":[0,12,"009739",12,23,"FEDD00",23,37,"012169",37,48,"FEDD00",48,60,"009739"]}}
    // Colombia {"seg":{"i":[0,30,"FFD100",30,45,"0072CE",45,60,"FF0000"]}}
    // Venezuela {"seg":{"i":[0,20,"FCE300",20,24,"003DA5",24,26,"FFFFFF",26,29,"003DA5",29,31,"FFFFFF",31,34,"003DA5",34,36,"FFFFFF",36,40,"003DA5",40,60,"FF0000"]}}
    //            check star size
    // Guyana {"seg":{"i":[0,18,"Ef3340",18,20,"000000",20,33,"FFD100",33,35,"FFFFFF",35,60,"009739"]}}
    // Panamá {"seg":{"i":[0,6,"FFFFFF",6,9,"0d44a8",9,15,"FFFFFF",15,30,"0d44a8",30,36,"FFFFFF",36,39,"DA121A",39,45,"FFFFFF",45,60,"DA121A"]}}
    // Costa Rica {"seg":{"i":[0,10,"00205B",10,20,"FFFFFF",20,40,"Ef3340",40,50,"FFFFFF",50,60,"00205B"]}}
    // Nicaragua {"seg":{"i":[0,20,"0067c6",20,28,"FFFFFF",28,32,"02de95",32,40,"FFFFFF",40,60,"0067c6"]}}
    // Honduras {"seg":{"i":[0,20,"0091d4",20,24,"FFFFFF",24,26,"0091d4",26,29,"FFFFFF",29,31,"0091d4",31,34,"FFFFFF",34,36,"0091d4",36,40,"FFFFFF",40,60,"0091d4"]}}
    // El Salvador {"seg":{"i":[0,20,"0047AB",20,26,"FFFFFF",26,34,"009900",34,40,"FFFFFF",40,60,"0047AB"]}}
    // Guatemala {"seg":{"i":[0,20,"4997D0",20,25,"FFFFFF",25,35,"269401",35,40,"FFFFFF",40,60,"4997D0"]}}
    // México {"seg":{"i":[0,20,"006341",20,27,"FFFFFF",27,33,"753100",33,40,"FFFFFF",40,60,"c8102E"]}}
    // Cuba {"seg":{"i":[0,8,"004B87",8,16,"FFFFFF",16,23,"004B87",23,29,"DA291C",29,31,á"FFFFFF",31,37,"DA291C",37,44,"004B87",44,52,"FFFFFF",52,60,"004B87"]}}
    // República Dominicana {"seg":{"i":[0,12,"002D62",12,16,"FFFFFF",16,28,"CE1126",28,32,"006300",32,44,"002D62",44,48,"FFFFFF",48,60,"CE1126"]}}
    // Jamaica {"seg":{"i":[0,12,"009B3A",12,24,"FED100",24,36,"000000",36,48,"FED100",48,60,"009B3A"]}}
    // Haití {"seg":{"i":[0,26,"00209F",26,34,"FFFFFF",34,60,"D21034"]}}
    // Las Antillas
    // United States {"seg":{"i":[0,6,"0A3161",6,8,"FFFFFF",8,14,"0A3161",14,16,"FFFFFF",16,22,"0A3161",22,24,"FFFFFF",24,30,"0A3161",30,35,"FFFFFF",35,40,"B31942",40,45,"FFFFFF",45,50,"B31942",50,55,"FFFFFF",55,60,"B31942"]}}
    // Canadá {"seg":{"i":[0,20,"D80621",20,25,"FFFFFF",25,35,"D80621",35,40,"FFFFFF",40,60,"D80621"]}}
    // Puerto Rico {"seg":{"i":[0,8,"Ce0000",8,16,"FFFFFF",16,23,"Ce0000",23,29,"33adff",29,31,"FFFFFF",31,37,"33adff",37,44,"Ce0000",44,52,"FFFFFF",52,60,"Ce0000"]}}


  }
