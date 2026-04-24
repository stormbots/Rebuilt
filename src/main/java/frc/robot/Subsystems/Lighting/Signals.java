// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Lighting;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
/** Add your docs here. */
public class Signals extends SubsystemBase {
  double[] ownedSegments=new double[]{0,1,2,3};
  LedSegment right = new LedSegment(0, 2, 22, false);
  LedSegment center = new LedSegment(1, 23, 40, false);
  LedSegment left = new LedSegment(2, 40, 59, false);

  // LedSegment backRight = new LedSegment(3, 0, 0, false);
  // LedSegment back = new LedSegment(4, 0, 0, false);
  // LedSegment backLeft = new LedSegment(5, 0, 0, false);
  
  public Signals(){
    right.setDefaultCommand(right.showAllianceColorInteresting().ignoringDisable(true));
    center.setDefaultCommand(center.showAllianceColorInteresting().ignoringDisable(true));
    left.setDefaultCommand(left.showAllianceColorInteresting().ignoringDisable(true));

    // backRight.setDefaultCommand(backRight.showAllianceColorInteresting().ignoringDisable(true));
    // back.setDefaultCommand(back.showAllianceColorInteresting().ignoringDisable(true));
    // backLeft.setDefaultCommand(backLeft.showAllianceColorInteresting().ignoringDisable(true));
  }

 public Command hopperFull(){
  return Commands.sequence(
    right.solidColor(CustomColor.kYellow),
    center.solidColor(CustomColor.kYellow),
    left.solidColor(CustomColor.kYellow),

    // backRight.solidColor(CustomColor.kYellow),
    // back.solidColor(CustomColor.kYellow),
    // backLeft.solidColor(CustomColor.kYellow),

    Commands.waitSeconds(0.5)
  );
 }

 public Command hopperLow(){
  return Commands.sequence(
    right.blinkSmooth(128,CustomColor.kYellow),
    center.blinkSmooth(128,CustomColor.kYellow),
    left.blinkSmooth(128,CustomColor.kYellow),

    // backRight.blinkSmooth(128,CustomColor.kYellow),
    // back.blinkSmooth(128,CustomColor.kYellow),
    // backLeft.blinkSmooth(128,CustomColor.kYellow),

    Commands.waitSeconds(0.5)
  );
 }
  
 public Command shotNotOk(){
  return Commands.sequence(
    right.solidColor(CustomColor.kOrange),
    center.solidColor(CustomColor.kOrange),
    left.solidColor(CustomColor.kOrange),

    // backRight.solidColor(CustomColor.kOrange),
    // back.solidColor(CustomColor.kOrange),
    // backLeft.solidColor(CustomColor.kOrange),

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

    // backRight.solidColor(CustomColor.kWhite),
    // Commands.runOnce(()->backRight.setBrightness(200),backRight),
    // back.solidColor(CustomColor.kWhite),
    // Commands.runOnce(()->back.setBrightness(200),back),
    // backLeft.solidColor(CustomColor.kWhite),
    // Commands.runOnce(()->backLeft.setBrightness(200),backLeft),

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

    // backRight.blinkSmooth(128,CustomColor.kWhite),
    // Commands.runOnce(()->backRight.setBrightness(200),backRight),
    // back.blinkSmooth(128,CustomColor.kWhite),
    // Commands.runOnce(()->back.setBrightness(200),back),
    // backLeft.blinkSmooth(128,CustomColor.kWhite),
    // Commands.runOnce(()->backLeft.setBrightness(200),backLeft),

    Commands.waitSeconds(1)
  );
 }


 public Command climbOkay(){
  return Commands.sequence(
    right.solidColor(CustomColor.kPink),
    center.solidColor(CustomColor.kPink),
    left.solidColor(CustomColor.kPink),

    // backRight.solidColor(CustomColor.kPink),
    // back.solidColor(CustomColor.kPink),
    // backLeft.solidColor(CustomColor.kPink),

    Commands.waitSeconds(0.5)
  );
 }

 public Command showVisionOkay(){
    Command command = Commands.sequence(
      right.solidColor(CustomColor.kGreen),
      center.solidColor(CustomColor.kGreen),
      left.solidColor(CustomColor.kGreen),

      // backRight.solidColor(CustomColor.kGreen),
      // back.solidColor(CustomColor.kGreen),
      // backLeft.solidColor(CustomColor.kGreen),

      Commands.waitSeconds(1)
    );
    return command.ignoringDisable(true);
  }

  public Command reboot(){
    Command command = Commands.sequence(
      Commands.runOnce(()->{}, right),
      Commands.runOnce(()->{}, center),
      Commands.runOnce(()->{}, left)

      // Commands.runOnce(()->{}, backRight),
      // Commands.runOnce(()->{}, back),
      // Commands.runOnce(()->{}, backLeft)
    );
    return command.ignoringDisable(true);
  }

  public Command automaticShot(){
    return Commands.sequence(
      right.automaticShot(),
      center.automaticShot(),
      left.automaticShot()

      // backRight.automaticShot(),
      // back.automaticShot(),
      // backLeft.automaticShot()
    );
  }

  public Command manualShot(){
    return Commands.sequence(
      right.manualShot(),
      center.manualShot(),
      left.manualShot()

      // backRight.manualShot(),
      // back.manualShot(),
      // backLeft.manualShot()
    );
  }

  public Command autoClimb(){
    Command command = Commands.sequence(
      right.autoClimb(),
      center.autoClimb(),
      left.autoClimb()

      // backRight.autoClimb(),
      // back.autoClimb(),
      // backLeft.autoClimb()
    );
    return command.ignoringDisable(true);
  }

  public Command wrongShot(){
    return Commands.sequence(
    right.solidColor(CustomColor.kYellow),
    center.solidColor(CustomColor.kYellow),
    left.solidColor(CustomColor.kYellow)

    // backRight.solidColor(CustomColor.kYellow),
    // back.solidColor(CustomColor.kYellow),
    // backLeft.solidColor(CustomColor.kYellow)
  );
  }

}


