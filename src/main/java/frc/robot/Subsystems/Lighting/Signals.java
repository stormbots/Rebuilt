// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Lighting;

import java.util.HashMap;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
/** Add your docs here. */
public class Signals extends SubsystemBase {

  double[] ownedSegments=new double[]{0,1,2,3};
  public WLED wled = new WLED();
  LedSegment segment = new LedSegment(0, 0, 60, false);
  LedSegment seg2 = new LedSegment(1, 60, 120, false);

  public Signals(){
    seg2.setDefaultCommand(seg2.seguimosAquí());
    segment.setDefaultCommand(showAllianceColorBoring(segment));
  }

  private Command showAllianceColorBoring(LedSegment segment){
    HashMap<Optional<Alliance>,Command> map = new HashMap<>();
    map.put(Optional.empty(), segment.solidColor(CustomColor.kPurple));
    map.put(Optional.of(Alliance.Blue), segment.solidColor(CustomColor.kBlue));
    map.put(Optional.of(Alliance.Red),segment.solidColor(CustomColor.kRed));
    return Commands.select(map, ()->DriverStation.getAlliance());
  }

 public Command hopperFull(LedSegment segment){
    return Commands.sequence(
      segment.solidColor(CustomColor.kYellow),
      segment.idle()
    );
 }

 public Command hopperLow(LedSegment segment){
    return Commands.sequence(
      segment.blink(CustomColor.kYellow,230),
      segment.idle()
    );
 }
  
}


