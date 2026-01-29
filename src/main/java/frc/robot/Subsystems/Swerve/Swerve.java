// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import java.io.File;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import frc.robot.lib.BLine.*;
import edu.wpi.first.math.controller.PIDController;


public class Swerve extends SubsystemBase {

  final double maximumSpeed = 2.0;

  SwerveDrive swerveDrive; 

  Field2d odometryField = new Field2d();


  /** Creates a new SwerveSubsystem. */
  public Swerve() {

    //this needs to be changed once final frame is decided
    var name = Preferences.getString("BotName", "compbot");
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),name);
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

  static class SwerveInputs{
    /** Positive meaning away from driver station */
    double tx=0;
    /** Positive meaning up from driver station */
    double ty=0;
    /** rotation, positive ccw*/
    double r=0;
    /** Zero out all inputs for this input set  */
    public void clear(){this.tx=0;this.ty=0;this.r=0;}
    /** Add another input to this one */
    public SwerveInputs add(SwerveInputs other){
      this.tx+=other.tx;
      this.ty+=other.ty;
      this.r+=other.r;
      return this;
    }
  }

  SwerveInputs driverInputs = new SwerveInputs();
  SwerveInputs fieldInputs = new SwerveInputs();
  SwerveInputs autoInputs = new SwerveInputs();

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    swerveDrive.updateOdometry();

    odometryField.setRobotPose(swerveDrive.getPose());

    var inputs = new SwerveInputs()
    .add(driverInputs)
    .add(fieldInputs)
    .add(autoInputs)
    ;
    if(DriverStation.isDisabled())inputs.clear();

    swerveDrive.drive(
        new Translation2d(
          inputs.tx * swerveDrive.getMaximumChassisVelocity(),
          inputs.ty * swerveDrive.getMaximumChassisVelocity()
        ),
        inputs.r * swerveDrive.getMaximumChassisAngularVelocity(),
        true,
        false 
      );
  }

  public Command addDriverInputs(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX){
    return Commands.either(
      run(()->{
        driverInputs.tx = 1 * translationX.getAsDouble();
        driverInputs.ty = 1 * translationY.getAsDouble();
        driverInputs.r = angularRotationX.getAsDouble();
      }), 
      run(()->{
        driverInputs.tx = -1 * translationX.getAsDouble();
        driverInputs.ty = -1 * translationY.getAsDouble();
        driverInputs.r = angularRotationX.getAsDouble();
      }),
      ()->DriverStation.getAlliance().equals(Optional.of(Alliance.Blue))
    )
    .finallyDo(driverInputs::clear)
    ;
  }

  public Command addAutoInputs(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX){
    return Commands.either(
      run(()->{
        autoInputs.tx = 1 * translationX.getAsDouble();
        autoInputs.ty = 1 * translationY.getAsDouble();
        driverInputs.r = angularRotationX.getAsDouble();
      }), 
      run(()->{
        autoInputs.tx = -1 * translationX.getAsDouble();
        autoInputs.ty = -1 * translationY.getAsDouble();
        autoInputs.r = angularRotationX.getAsDouble();
      }),
      ()->DriverStation.getAlliance().equals(Optional.of(Alliance.Blue))
    )
    .finallyDo(autoInputs::clear)
    ;
  }

  public Command zeroGyro(){
    return Commands.runOnce(swerveDrive::zeroGyro);
  }

  //For other subsystems/files
  public Pose2d getSwervePose(){
    return swerveDrive.getPose();
  }

  public void addVisionMeasurement(Pose2d pose2d, double timestamp, Matrix<N3, N1> STD_DEVS){
    swerveDrive.addVisionMeasurement(pose2d, timestamp, STD_DEVS);
  }

  public ChassisSpeeds getChassisSpeedsRobotRelative(){
    return swerveDrive.getRobotVelocity();
  }

  public ChassisSpeeds getChassisSpeedsFieldRelative(){
    return swerveDrive.getFieldVelocity();
  }


}