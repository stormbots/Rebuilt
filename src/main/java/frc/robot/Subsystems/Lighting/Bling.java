// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Lighting;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Bling extends SubsystemBase {
  /** Glorious Gratuitous Glow */
  public WLED wled = new WLED();
  public Bling() {}

  // @Override
  // public void periodic() {
  //   // This method will be called once per scheduler run
  // }

  // public Command pride(LedSegment segment){
  //   CustomColor pink = new CustomColor(255, 28, 206);
  //   CustomColor white = new CustomColor(255, 255, 255);
  //   CustomColor babyBlue = new CustomColor(38, 14, 255);
  //   CustomColor red = new CustomColor(255, 0, 0);
  //   CustomColor orange = new CustomColor(255, 140, 0);
  //   CustomColor yellow = new CustomColor(255, 215, 0);
  //   CustomColor green = new CustomColor(0, 255, 0);
  //   CustomColor blue = new CustomColor(0, 0, 255);
  //   CustomColor purple = new CustomColor(255, 0, 255);
  //   CustomColor black = new CustomColor(0, 0, 0);
  //   CustomColor grey = new CustomColor(119, 119, 119);
  //   CustomColor magenta = new CustomColor(255, 0, 111);
  //   CustomColor lightGreen = new CustomColor(159, 255, 133);
  //   double StartTime = Timer.getFPGATimestamp();
  //   double duration = 5;
    
  //   return new RunCommand(()->{
  //     double time = (Timer.getFPGATimestamp()-StartTime)%(duration*7);
      
  //     if (time < duration){
  //       setState(
  //       "{\"seg\":["+
  //       "{"+
  //         "\"id\":" + segment.id + ","+
  //         "\"i\":[0,"+segment.length/5+",\""+ 
  //         babyBlue.getHex() +"\","+ 
  //         segment.length/5+","+ (segment.length/5)*2 +",\""+
  //         pink.getHex() +"\","+
  //         (segment.length/5)*2+","+(segment.length/5)*3+",\""+
  //         white.getHex() +"\","+
  //         (segment.length/5)*3+","+(segment.length/5)*4+",\""+
  //         pink.getHex() +"\","+
  //         (segment.length/5)*4+","+segment.length+",\""+
  //         babyBlue.getHex() +
  //         "\"]"+
  //       "}"+
  //       "]}");
  //     }

  //     else if (time < duration*2){
  //       setState(
  //       "{\"seg\":["+
  //       "{"+
  //         "\"id\":" + segment.id + ","+
  //         "\"i\":[0,"+segment.length/6+",\""+ 
  //         red.getHex() +"\","+ 
  //         segment.length/6+","+ (segment.length/6)*2 +",\""+
  //         orange.getHex() +"\","+
  //         (segment.length/6)*2+","+(segment.length/6)*3+",\""+
  //         yellow.getHex() +"\","+
  //         (segment.length/6)*3+","+(segment.length/6)*4+",\""+
  //         green.getHex() +"\","+
  //         (segment.length/6)*4+","+(segment.length/6)*5+",\""+
  //         blue.getHex() +"\","+
  //         (segment.length/6)*5+","+segment.length + ",\""+
  //         purple.getHex() +
  //         "\"]"+
  //       "}"+
  //       "]}");
  //     }

  //     else if (time < duration*3){
  //       setState(
  //       "{\"seg\":["+
  //       "{"+
  //         "\"id\":" + segment.id + ","+
  //         "\"i\":[0,"+segment.length/4+",\""+ 
  //         yellow.getHex() +"\","+ 
  //         segment.length/4+","+ (segment.length/4)*2 +",\""+
  //         white.getHex() +"\","+
  //         (segment.length/4)*2+","+(segment.length/4)*3+",\""+
  //         purple.getHex() +"\","+
  //         (segment.length/4)*3+","+segment.length+",\""+
  //         black.getHex() +"\","+
  //         "\"]"+
  //       "}"+
  //       "]}");
  //     }

  //     else if (time < duration*4){
  //       setState(
  //       "{\"seg\":["+
  //       "{"+
  //         "\"id\":" + segment.id + ","+
  //         "\"i\":[0,"+(segment.length/5)*2+",\""+ 
  //         magenta.getHex() +"\","+ 
  //         (segment.length/5)*2+","+(segment.length/5)*3+",\""+
  //         purple.getHex() +"\","+
  //         (segment.length/5)*3+","+segment.length+",\""+
  //         blue.getHex() +
  //         "\"]"+
  //       "}"+
  //       "]}");
  //     }

  //     else if (time < duration*5){
  //       int fraction = segment.length/18;
  //       if (fraction == 0){
  //         fraction = 1;
  //       }
  //       setState(
  //       "{\"seg\":["+
  //       "{"+
  //         "\"id\":" + segment.id + ","+
  //         "\"i\":[0,"+segment.length/3+",\""+ 
  //         yellow.getHex() +"\","+ 
  //         segment.length/3+","+ fraction*7 +",\""+
  //         purple.getHex() +"\","+
  //         fraction*7+","+fraction*11+",\""+
  //         yellow.getHex() +"\","+
  //         fraction*11+","+(segment.length/3)*2+",\""+
  //         purple.getHex() +"\","+
  //         (segment.length/3)*2+","+segment.length+",\""+
  //         yellow.getHex() +
  //         "\"]"+
  //       "}"+
  //       "]}");
  //     }

  //     else if (time < duration*6){
  //       setState(
  //       "{\"seg\":["+
  //       "{"+
  //         "\"id\":" + segment.id + ","+
  //         "\"i\":[0,"+segment.length/4+",\""+ 
  //         black.getHex() +"\","+ 
  //         segment.length/4+","+ (segment.length/4)*2 +",\""+
  //         grey.getHex() +"\","+
  //         (segment.length/4)*2+","+(segment.length/4)*3+",\""+
  //         white.getHex() +"\","+
  //         (segment.length/4)*3+","+segment.length+",\""+
  //         purple.getHex() +"\","+
  //         "\"]"+
  //       "}"+
  //       "]}");
  //     }

  //     else {
  //       setState(
  //       "{\"seg\":["+
  //       "{"+
  //         "\"id\":" + segment.id + ","+
  //         "\"i\":[0,"+segment.length/5+",\""+ 
  //         green.getHex() +"\","+ 
  //         segment.length/5+","+ (segment.length/5)*2 +",\""+
  //         lightGreen.getHex() +"\","+
  //         (segment.length/5)*2+","+(segment.length/5)*3+",\""+
  //         white.getHex() +"\","+
  //         (segment.length/5)*3+","+(segment.length/5)*4+",\""+
  //         grey.getHex() +"\","+
  //         (segment.length/5)*4+","+segment.length+",\""+
  //         black.getHex() +
  //         "\"]"+
  //       "}"+
  //       "]}");
  //     }
  //   }, this).handleInterrupt(()->setState("{\"seg\":[{\"id\":"+segment.id+",\"frz\":false}]}"));
  // }

}
