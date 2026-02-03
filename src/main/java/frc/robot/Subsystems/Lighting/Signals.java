// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Lighting;

import java.util.HashMap;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        segment.setDefaultCommand(showAllianceColorBoring(segment).ignoringDisable(true));
        // segment.setDefaultCommand(hopperLow(segment));
    }




  private Command showAllianceColorBoring(LedSegment segment){
    HashMap<Optional<Alliance>,Command> map = new HashMap<>();
    map.put(Optional.empty(), segment.solidColor(wled.purple));
    map.put(Optional.of(Alliance.Blue), segment.solidColor(wled.blue));
    map.put(Optional.of(Alliance.Red),segment.solidColor(wled.red));
    return Commands.select(map, ()->DriverStation.getAlliance());
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


