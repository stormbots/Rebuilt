// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Lighting;

import java.util.HashMap;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
/** Add your docs here. */
public class Signals extends SubsystemBase {

  double[] ownedSegments=new double[]{0,1,2,3};
  public WLED wled = new WLED(new SerialPort(115200, Port.kUSB1));
  LedSegment seg = new LedSegment(0, 0, 60, false);
  LedSegment seg2 = new LedSegment(1, 60, 120, false);
  List<LedSegment> signalSegments = List.of(seg,seg2);

  
  public Signals(){
    seg.setDefaultCommand(showAllianceColorBoring(seg));
    seg2.setDefaultCommand(showAllianceColorBoring(seg2));
  }

  private Command showAllianceColorBoring(LedSegment segment){
    HashMap<Optional<Alliance>,Command> map = new HashMap<>();
    map.put(Optional.empty(), segment.solidColor(CustomColor.kPurple));
    map.put(Optional.of(Alliance.Blue), segment.solidColor(CustomColor.kBlue));
    map.put(Optional.of(Alliance.Red),segment.solidColor(CustomColor.kRed));
    return Commands.select(map, ()->DriverStation.getAlliance());
  }

 public Command hopperFull(){
  return Commands.sequence(
    seg.solidColor(CustomColor.kYellow),
    seg2.solidColor(CustomColor.kYellow),
    Commands.waitSeconds(0.5)
  );
 }

 public Command hopperLow(){
  return Commands.sequence(
    seg.blink(CustomColor.kYellow,230),
    seg2.blink(CustomColor.kYellow,230),
    Commands.waitSeconds(1)
  );
 }
  
 public Command shotNotOk(){
  return Commands.sequence(
    seg.solidColor(CustomColor.kOrange),
    seg2.solidColor(CustomColor.kOrange),
    Commands.waitSeconds(0.5)
  );
 }

 public Command shiftChange(){
  return Commands.sequence(
    seg.solidColor(CustomColor.kWhite),
    seg2.solidColor(CustomColor.kWhite),
    Commands.waitSeconds(0.5)
  );
 }

 public Command climbOkay(){
  return Commands.sequence(
    seg.solidColor(CustomColor.kPink),
    seg2.solidColor(CustomColor.kPink),
    Commands.waitSeconds(0.5)
  );
 }

}


