// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Lighting;

import java.awt.Color;

import org.dyn4j.geometry.Segment;

import com.ctre.phoenix6.controls.SolidColor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Lighting.WLED.CustomColor;
import frc.robot.Subsystems.Lighting.WLED.LedSegment;

/** Add your docs here. */
public class Signals extends SubsystemBase {

    double[] ownedSegments=new double[]{0,1,2,3};
    public WLED wled = new WLED();

    // private WLED.LedSegment segment = wled.new LedSegment(0, 0, 30, false);

    LedSegment segment = wled.getLedSegment(0, 0, 30, false);


    public Signals(){
        segment.setDefaultCommand(hopperFull());
        // setDefaultCommand(hopperFull());
    }


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



//Must haves
// hopperfull
// hopperlow
// shift/activated
// team color/solid color
// shoot status?

 public Command hopperFull(){
    return Commands.sequence(
        segment.solidColor(wled.yellow),
        idle()
    );
 }


 /**
  * seg0,seg1,seg2,seg3
  */

  
}


