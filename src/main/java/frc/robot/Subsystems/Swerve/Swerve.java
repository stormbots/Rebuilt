// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;

import java.io.File;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
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

public class Swerve extends SubsystemBase {

  final double maximumSpeed = 2.0;
  public SwerveDrive swerveDrive; 
  Field2d odometryField = new Field2d();
  private boolean isOnTargetAngle = true;
  /** Creates a new SwerveSubsystem. */
  public Swerve() {
    var botname = Preferences.getString("BotName", "compbot");
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),botname);
    try
    {
      swerveDrive = new SwerveParser(swerveJsonDirectory)
      .createSwerveDrive(maximumSpeed, new Pose2d(1,1,new Rotation2d()));
    } catch (Exception e)
    {
      System.err.println("Could not find robot config for " + botname);
      throw new RuntimeException(e);
    }  

    swerveDrive.setMotorIdleMode(true);
    swerveDrive.setModuleStateOptimization(true);
    swerveDrive.setCosineCompensator(false);
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    SmartDashboard.putData("odometryField", odometryField);
  }

  /** Represent Inputs as a proportion to the drive trains maximum capability */
  public static class SwerveInputs{
    /** Positive meaning away from driver station */
    public double tx=0;
    /** Positive meaning up from driver station */
    public double ty=0;
    /** rotation, positive ccw*/
    public double r=0;
    /** Zero out all inputs for this input set  */
    public void clear(){this.tx=0;this.ty=0;this.r=0;}
    /** Add another input to this one */
    public SwerveInputs add(SwerveInputs other){
      this.tx+=other.tx;
      this.ty+=other.ty;
      this.r+=other.r;
      return this;
    }

    /** Create an input using relative chassis outputs [+/-1] */
    public static SwerveInputs fromRelativePower(double tx, double ty, double rotation){
      var inputs = new SwerveInputs();
      inputs.tx = tx;
      inputs.ty = ty;
      inputs.r = rotation;
      return inputs;
    }

    /** Update the input, returning itself */
    public SwerveInputs addtx(Double tx){ this.tx +=tx; return this; }
    /** Update the input, returning itself */
    public SwerveInputs addty(Double tx){ this.ty +=ty; return this; }
    /** Update the input, returning itself */
    public SwerveInputs addr(Double tx){ this.r +=r; return this; }
  }

  SwerveInputs primaryInputs = new SwerveInputs();
  SwerveInputs fieldInputs = new SwerveInputs();
  SwerveInputs secondaryInputs = new SwerveInputs();

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    swerveDrive.updateOdometry();
    //Log the pose to allow AdvantageScope to work properly
    DogLog.log("Swerve/pose", swerveDrive.getPose());

    odometryField.setRobotPose(swerveDrive.getPose());
    var inputs = new SwerveInputs()
    .add(primaryInputs)
    .add(fieldInputs)
    .add(secondaryInputs)
    ;

    //Don't generate output when off
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

    //Now that we've read the inputs, clear them to prevent potential stale data
    primaryInputs.clear();
    fieldInputs.clear();
    secondaryInputs.clear();

    SmartDashboard.putNumber("swerve/primaryInput/tx", primaryInputs.tx);
    SmartDashboard.putNumber("swerve/primaryInput/ty", primaryInputs.ty);
    SmartDashboard.putNumber("swerve/primaryInput/r", primaryInputs.r);

    SmartDashboard.putNumber("swerve/secondaryInput/tx", primaryInputs.tx);
    SmartDashboard.putNumber("swerve/secondaryInput/ty", primaryInputs.ty);
    SmartDashboard.putNumber("swerve/secondaryInput/r", primaryInputs.r);

    SmartDashboard.putNumber("swerve/anglegyro", swerveDrive.getGyro().getRotation3d().getAngle());
  }

  /** Own the subsystem and add dominant field-centric control */
  public Command setPrimaryInputs(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX){
    return Commands.either(
      run(()->{
        primaryInputs.tx = 1 * translationX.getAsDouble();
        primaryInputs.ty = 1 * translationY.getAsDouble();
        primaryInputs.r = angularRotationX.getAsDouble();
      }), 
      run(()->{
        primaryInputs.tx = -1 * translationX.getAsDouble();
        primaryInputs.ty = -1 * translationY.getAsDouble();
        primaryInputs.r = angularRotationX.getAsDouble();
      }),
      ()->DriverStation.getAlliance().equals(Optional.of(Alliance.Blue))
    )
    .finallyDo(primaryInputs::clear)
    ;
  }

  /** Special interface for path-planning, which needs a method interface
   * to build it's command.
   * @param translationX in percent of chassis power
   * @param translationY in percent of chassis power
   * @param angularRotationX in percent of chassis power
   */
  public void setPrimaryInputsVoid(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX){
    primaryInputs.tx = 1 * translationX.getAsDouble();
    primaryInputs.ty = 1 * translationY.getAsDouble();
    primaryInputs.r = angularRotationX.getAsDouble();
  }

  /** Add additional inputs for automatic actions like turning/aiming without disrupting primary input.
   * Does not claim subsystem.
   * @param translationX in percentage of chassis power
   * @param translationY in percent chassis power
   * @param angularRotationX in percent chassis power
   * @return
   */
  public Command addSecondaryInputs(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX){
    return Commands.either(
      Commands.run(()->{
        secondaryInputs.tx += 1 * translationX.getAsDouble();
        secondaryInputs.ty += 1 * translationY.getAsDouble();
        secondaryInputs.r += angularRotationX.getAsDouble();
      }), 
      Commands.run(()->{
        secondaryInputs.tx += -1 * translationX.getAsDouble();
        secondaryInputs.ty += -1 * translationY.getAsDouble();
        secondaryInputs.r += angularRotationX.getAsDouble();
      }),
      ()->DriverStation.getAlliance().equals(Optional.of(Alliance.Blue))
    )
    .finallyDo(secondaryInputs::clear)
    ;
  }

  /** Provide inputs generated by field position; Does not claim subsystem! */
  public Command addFieldInput(Supplier<SwerveInputs> inputs){
    return Commands.run(()->{
      fieldInputs = inputs.get();
    });
  };

  public Command zeroGyro(){
    return Commands.runOnce(swerveDrive::zeroGyro);
  }

  public Command testZeroPose(){
    return Commands.runOnce(()->swerveDrive.resetOdometry(new Pose2d()));
  }

  //For other subsystems/files
  public Pose2d getSwervePose(){
    return swerveDrive.getPose();
  }

  public ChassisSpeeds getChassisSpeeds(){
    return swerveDrive.getRobotVelocity();
  }

  public boolean isOnTargetAngle(){
    return isOnTargetAngle;
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

  private void pidToRotation(Rotation2d targetRot){
    double clamp = 2.0;
    // swerveDrive.setChassisSpeeds(new ChassisSpeeds(
    //   MathUtil.clamp(delta.getX()*transltionP,-clamp, clamp),
    //   MathUtil.clamp(delta.getY()*transltionP,-clamp,clamp),
    //   delta.getRotation().getRadians()*thetaP
    // ));

    secondaryInputs.r = targetRot.getDegrees()*1/90.0;

  }


  public Command turnToHeading(Supplier<Rotation2d> bearing){
    return Commands.run(()->{
      isOnTargetAngle = false;
      secondaryInputs.r = 0.3;
      var error = bearing.get().minus(swerveDrive.getPose().getRotation());
      // var error = swerveDrive.getPose().getRotation().minus(bearing);  
      double kp = 2.0 / 120.0; //90 degrees is 1 output
      double output = error.getDegrees()*kp;
      output = MathUtil.clamp(output, -1.0, 1.0);
      secondaryInputs.r = output;
      if(error.getDegrees() < 5.0){
        isOnTargetAngle = true;
      }
    }).finallyDo(()->isOnTargetAngle = false)
    ;
  }
}