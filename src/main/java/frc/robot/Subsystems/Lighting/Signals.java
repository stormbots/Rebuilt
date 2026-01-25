// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Lighting;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Lighting.WLED.CustomColor;
import frc.robot.Subsystems.Lighting.WLED.LedSegment;

/** Add your docs here. */
public class Signals {
    public Signals(){}

//     public Command hopperFull(LedSegment segment){
//     // return run(()->setState("{\"seg\":[{\"id\":" + segment.id + "\"col\":[[255,204,0]]}]}"));
//     CustomColor color = new CustomColor(255, 215, 0);
//     return solidColor(segment, color, 1);
//   }

//   public Command hopperLow(LedSegment segment){
//     // return run(()->setState("{\"seg\":[{\"id\":" + segment.id + "\"col\":[[255,204,0]]}]}"));
//     CustomColor color = new CustomColor(255, 215, 0);
//     return blink(segment, color, 225 ,1);
//   }

//   public Command showAllianceColorBoring(LedSegment segment){
//     return new RunCommand(()->{
//       var color = DriverStation.getAlliance();

//       if (color.isPresent()){}
//     }, this);
//   }

//   public Command showhopperFull(){
//     wled.segment(0).setPattern(1);
//     wled.segment(0).solidColor(color);
//     // wled.segment(0).setPattern(1,primaryColor,secondaryColor,tertiary,time,etc)
//     wled.pattern(segmet)

//   }


}


