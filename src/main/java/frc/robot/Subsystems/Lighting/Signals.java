// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Lighting;

import java.util.HashMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Lighting.WLED.LedSegment;

/** Add your docs here. */
public class Signals extends SubsystemBase {

    double[] ownedSegments=new double[]{0,1,2,3};
    public WLED wled = new WLED();
    DriverStation.Alliance lastColor;// = null;

    // private WLED.LedSegment segment = wled.new LedSegment(0, 0, 30, false);

    LedSegment segment = wled.getLedSegment(0, 0, 30, false);


    public Signals(){
        // setDefaultCommand(showAllianceColorBoring(segment));
        segment.setDefaultCommand(hopperLow(segment));
    }


//   public Command hopperLow(LedSegment segment){
//     // return run(()->setState("{\"seg\":[{\"id\":" + segment.id + "\"col\":[[255,204,0]]}]}"));
//     CustomColor color = new CustomColor(255, 215, 0);
//     return blink(segment, color, 225 ,1);
//   }

//   public Command showAllianceColorBoring(LedSegment segment){
//     // lastColor = Optional.empty();
//     Command command = doAllianceSelect(segment);
//     command.addRequirements(segment);

//     return command;
//   }

  private Command doAllianceSelect(LedSegment segment){
    // HashMap<Optional<Alliance>,Command> map = new HashMap<>();
    // map.put(Optional.empty(), segment.solidColor(wled.purple));
    // map.put(Optional.of(Alliance.Blue), segment.solidColor(wled.blue));
    // map.put(Optional.of(Alliance.Red),segment.solidColor(wled.red));

    HashMap<Alliance,Command> map = new HashMap<>();
    // map.put(Optional.empty(), segment.solidColor(wled.purple));
    map.put(Alliance.Blue, segment.solidColor(wled.blue));
    map.put(Alliance.Red,segment.solidColor(wled.red));

    return Commands.select(map, ()->DriverStation.getAlliance().orElse(Alliance.Blue));
  }



//Must haves
// hopperfull
// hopperlow
// shift/activated
// team color/solid color
// shoot status?

 public Command hopperFull(LedSegment segment){
    return Commands.sequence(
        segment.solidColor(wled.yellow),
        segment.idle()
    );
 }

 public Command hopperLow(LedSegment segment){
    return Commands.sequence(
        segment.blink(wled.yellow,230),
        segment.idle()
    );
 }

//  public Command test(){
//     // return Commands.
//     return Commands.
//  }

 /**
  * seg0,seg1,seg2,seg3
  */

  
}


