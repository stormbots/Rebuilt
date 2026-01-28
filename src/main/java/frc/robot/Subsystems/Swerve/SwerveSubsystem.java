// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.imu.NavXSwerve;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;


public class SwerveSubsystem extends SubsystemBase {

  final double maximumSpeed = 5.0;

  SwerveDrive swerveDrive; 

  Field2d odometryField = new Field2d();


  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {

    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"choppedbot");
    try
    {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed, new Pose2d());
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }  

    swerveDrive.setMotorIdleMode(true);
    swerveDrive.setModuleStateOptimization(true);
    swerveDrive.setCosineCompensator(false);

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    SmartDashboard.putData("odometryField", odometryField);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    swerveDrive.updateOdometry();

    odometryField.setRobotPose(swerveDrive.getPose());
  }

  public Command driveAllianceManagedCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX){
    return run(()->{
      //when is red, the translation x and y are flipped, FIX THIS LATER WE DONT HAVE A CONSTANTS YET
      // double allianceFlip = Constants.isRed.getAsBoolean() ? -1 : 1;
      double allianceFlip = 1.0;
      swerveDrive.drive(
        new Translation2d(
          -allianceFlip * translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
          -allianceFlip * translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()
        ),
        angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
        false,
        false 
      );
    });
  }
}