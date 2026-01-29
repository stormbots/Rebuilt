// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;


public class Pathing extends SubsystemBase {
  /** Creates a new Pathing. */
  Swerve swerveSubsystem;

  FollowPath.Builder pathBuilder;

  public Pathing(Swerve swerveSubsytem) {
    this.swerveSubsystem = swerveSubsytem;
    pathBuilder = new FollowPath.Builder(
    swerveSubsystem, 
    swerveSubsystem::getSwervePose, 
    swerveSubsystem::getChassisSpeedsRobotRelative, 
    this::setAutoInputs, 
    new PIDController(5.0, 0.0, 0.0),    // Translation PID
    new PIDController(3.0, 0.0, 0.0),    // Rotation PID
    new PIDController(2.0, 0.0, 0.0)     // Cross-track PID
    );
  }

  public Command followPath(Path path){
    return pathBuilder.build(path);
  }

  private void setAutoInputs(ChassisSpeeds robotRelative) {
    //Converting robot relative from bline for field relative inputs
    Rotation2d heading = swerveSubsystem.getSwervePose().getRotation(); 
    ChassisSpeeds fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelative, heading);
    swerveSubsystem.addAutoInputs(()->fieldRelative.vxMetersPerSecond/2.0, ()->fieldRelative.vyMetersPerSecond/2.0, ()->fieldRelative.omegaRadiansPerSecond);
}
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}
