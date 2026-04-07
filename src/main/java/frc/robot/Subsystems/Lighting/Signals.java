// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Lighting;

import java.util.HashMap;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
/** Add your docs here. */
public class Signals extends SubsystemBase {
  double[] ownedSegments=new double[]{0,1,2,3};
  LedSegment right = new LedSegment(0, 2, 22, false);
  LedSegment center = new LedSegment(1, 23, 40, false);
  LedSegment left = new LedSegment(2, 40, 59, false);

  List<LedSegment> signalSegments = List.of(right,center);

  
  public Signals(){
    right.setDefaultCommand(right.showAllianceColorInteresting().ignoringDisable(true));
    center.setDefaultCommand(center.showAllianceColorInteresting().ignoringDisable(true));
    left.setDefaultCommand(left.showAllianceColorInteresting().ignoringDisable(true));
  }

  public Command showAllianceColor(){
    return Commands.sequence(showAllianceColorBoring(right),showAllianceColorBoring(center));
  }
  
  public Command showAllianceColorIntersingUnchecked(){
    return Commands.sequence(right.showAllianceColorUnchecked(),center.showAllianceColorUnchecked());
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
    right.solidColor(CustomColor.kYellow),
    center.solidColor(CustomColor.kYellow),
    left.solidColor(CustomColor.kYellow),
    Commands.waitSeconds(0.5)
  );
 }

 public Command hopperLow(){
  return Commands.sequence(
    right.blinkSmooth(128,CustomColor.kYellow),
    center.blinkSmooth(128,CustomColor.kYellow),
    left.blinkSmooth(128,CustomColor.kYellow),
    Commands.waitSeconds(0.5)
  );
 }
  
 public Command shotNotOk(){
  return Commands.sequence(
    right.solidColor(CustomColor.kOrange),
    center.solidColor(CustomColor.kOrange),
    left.solidColor(CustomColor.kOrange),
    Commands.waitSeconds(0.5)
  );
 }

 public Command shiftStart(){
  return Commands.sequence(
    right.solidColor(CustomColor.kWhite),
    Commands.runOnce(()->right.setBrightness(200),right),
    center.solidColor(CustomColor.kWhite),
    Commands.runOnce(()->center.setBrightness(200),center),
    left.solidColor(CustomColor.kWhite),
    Commands.runOnce(()->left.setBrightness(200),left),
    Commands.waitSeconds(1)
  );
 }

 public Command shiftEnd(){
  return Commands.sequence(
    right.blinkSmooth(128,CustomColor.kWhite),
    Commands.runOnce(()->right.setBrightness(200),right),
    center.blinkSmooth(128,CustomColor.kWhite),
    Commands.runOnce(()->center.setBrightness(200),center),
    left.blinkSmooth(128,CustomColor.kWhite),
    Commands.runOnce(()->left.setBrightness(200),left),
    Commands.waitSeconds(1)
  );
 }


 public Command climbOkay(){
  return Commands.sequence(
    right.solidColor(CustomColor.kPink),
    center.solidColor(CustomColor.kPink),
    left.solidColor(CustomColor.kPink),
    Commands.waitSeconds(0.5)
  );
 }

 public Command showVisionOkay(){
    Command command = Commands.sequence(
      right.solidColor(CustomColor.kGreen),
      center.solidColor(CustomColor.kGreen),
      left.solidColor(CustomColor.kGreen),
      Commands.waitSeconds(1)
    );
    return command.ignoringDisable(true);
  }

  public Command reboot(){
    Command command = Commands.sequence(
      Commands.runOnce(()->{}, right),
      Commands.runOnce(()->{}, center),
      Commands.runOnce(()->{}, left)
    );
    return command.ignoringDisable(true);
  }

  public Command automaticShot(){
    return Commands.sequence(
    right.solidColor(CustomColor.kGreen),
    center.solidColor(CustomColor.kGreen),
    left.solidColor(CustomColor.kGreen)
  );
  }

  public Command manualShot(){
    return Commands.sequence(
    right.solidColor(CustomColor.kPurple),
    center.solidColor(CustomColor.kPurple),
    left.solidColor(CustomColor.kPurple)
  );
  }

  public Command wrongShot(){
    return Commands.sequence(
    right.solidColor(CustomColor.kYellow),
    center.solidColor(CustomColor.kYellow),
    left.solidColor(CustomColor.kYellow)
  );
  }

}


