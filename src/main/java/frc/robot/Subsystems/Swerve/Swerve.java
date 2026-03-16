// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.Subsystems.Swerve;


import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.studica.frc.AHRS;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;


public class Swerve extends SubsystemBase {


  final double maximumSpeed = 5.0;
  public SwerveDrive swerveDrive;
  Field2d odometryField = new Field2d();
  private boolean isOnTargetAngle = true;
  private boolean isOnTargetTranslate = true;


  private AHRS navx;


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


    navx = (AHRS) swerveDrive.getGyro().getIMU();
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


    SmartDashboard.putNumber("swerve/secondaryInput/tx", secondaryInputs.tx);
    SmartDashboard.putNumber("swerve/secondaryInput/ty", secondaryInputs.ty);
    SmartDashboard.putNumber("swerve/secondaryInput/r", secondaryInputs.r);


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
    ;
  }


  /** Provide inputs generated by field position; Does not claim subsystem! */
  public Command addFieldInput(Supplier<SwerveInputs> inputs){
    return Commands.run(()->{
      fieldInputs = inputs.get();
    });
  };




 
  public Command isCalibrating(){
    return Commands.idle().until(()->(navx.isCalibrating()== false));
  }  


  public Command zeroGyro(){
    return Commands.runOnce(swerveDrive::zeroGyro);
  }


  public Command testZeroPose(){
    return Commands.runOnce(()->swerveDrive.resetOdometry(new Pose2d(4.0, 0.8, new Rotation2d())));
  }


  //For other subsystems/files
  public Pose2d getSwervePose(){
    return swerveDrive.getPose();
  }


  public ChassisSpeeds getChassisSpeeds(){
    return swerveDrive.getRobotVelocity();
  }

  public ChassisSpeeds getFieldRelativeChassisSpeeds(){
    return swerveDrive.getRobotVelocity();
  }


  public boolean isOnTargetAngle(){
    return isOnTargetAngle;
  }


  public boolean isOnTargetTranslate(){
    return isOnTargetTranslate;
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
      var error = bearing.get().minus(swerveDrive.getPose().getRotation());
      // var error = swerveDrive.getPose().getRotation().minus(bearing);  
      double kp = 2.0 / 120.0; //90 degrees is 1 output
      double output = error.getDegrees()*kp;
      output = MathUtil.clamp(output, -1.0, 1.0);
      secondaryInputs.r = output;
      if(Math.abs(error.getDegrees()) < 5.0){
        isOnTargetAngle = true;
      }
    }).finallyDo(()->{
      isOnTargetAngle = false;
      secondaryInputs.r=0;
    })
    ;
  }

  public Command verifyAngleTargetPass(Supplier<Rotation2d> bearing){
    return Commands.run(()->{
      isOnTargetAngle = false;
      var error = bearing.get().minus(swerveDrive.getPose().getRotation());

      if(Math.abs(error.getDegrees()) < 85.0){
        isOnTargetAngle = true;
      }
    }).finallyDo(()->{
      isOnTargetAngle = false;
    });
  }
  public Command stop(){
    return Commands.run(()->{
    primaryInputs.tx = 0;
    primaryInputs.ty = 0;
    primaryInputs.r = 0;
    secondaryInputs.tx = 0;
    secondaryInputs.ty = 0;
    secondaryInputs.r = 0;});
  }
  public Command turnToHeadingWithinTurretRange(Supplier<Rotation2d> bearing){
    return Commands.run(()->{
      isOnTargetAngle = false;
      var error = bearing.get().minus(swerveDrive.getPose().getRotation());
      // var error = swerveDrive.getPose().getRotation().minus(bearing);  
      double kp = 2.0 / 120.0; //90 degrees is 1 output
      double output = error.getDegrees()*kp;
      output = MathUtil.clamp(output, -2.0, 2.0);
      if(Math.abs(error.getDegrees()) > 5.0){
        secondaryInputs.r = output;
      }
      else{
        isOnTargetAngle = true;
        secondaryInputs.r = 0.0;
      }
    }).finallyDo(()->{
      isOnTargetAngle = false;
      secondaryInputs.r=0;
    })
    ;
  }
  public Command pidToPose(Supplier<Pose2d> targetPoseSupplier){
    return pidToPose(targetPoseSupplier,maximumSpeed,Inches.of(5));
  }


  public Command pidToPose(Supplier<Pose2d> targetPoseSupplier, double maxVelocityMPS, Distance tolerance){
    return Commands.run(() -> {
      isOnTargetTranslate = false;
      isOnTargetAngle = false;

      Pose2d currentPose = swerveDrive.getPose();
      Pose2d targetPose = targetPoseSupplier.get();
      double errorX = targetPose.getX() - currentPose.getX();
      double errorY = targetPose.getY() - currentPose.getY();


      Rotation2d angleError = targetPose.getRotation().minus(currentPose.getRotation());


      double kPTranslation = 1.2;
      double kPRotation = 1.0 / 90.0 * 2.0 * 0.5; // 90 deg -> about 2 output before clamp


      double xOutput = errorX * kPTranslation;
      double yOutput = errorY * kPTranslation;
      double rOutput = angleError.getDegrees() * kPRotation;

      // double maxMagnitude = Math.hypot(xOutput, yOutput) * maxVelocityMPS/maximumSpeed; 
      swerveDrive.setMaximumAllowableSpeeds(maxVelocityMPS, 720);
      secondaryInputs.tx = xOutput;
      secondaryInputs.ty = yOutput;
      secondaryInputs.r = rOutput;

      
      if (Math.hypot(errorX, errorY) < tolerance.in(Meter)) {
        isOnTargetTranslate = true;
      }

      if (Math.abs(angleError.getDegrees()) < 5.0) {
        isOnTargetAngle = true;
      }
    }).finallyDo(() -> {
      swerveDrive.setMaximumAllowableSpeeds(4.0, 720);
      isOnTargetAngle = false;
      isOnTargetTranslate = false;
      secondaryInputs.tx = 0.0;
      secondaryInputs.ty = 0.0;
      secondaryInputs.r = 0.0;
    });
}


// These are all super gross things needed for the PID command, please ignore, and have to be set up
//Can't be set as proper local commands, so it's a mess 

// These are in percent output at X meters
PIDController xpid = new PIDController(
  1/0.5, 0, 0
  // new Constraints(swerveDrive.getMaximumChassisVelocity(), 5)
);
PIDController ypid = new PIDController(
  1/0.5, 0, 0
  // new Constraints(swerveDrive.getMaximumChassisVelocity(), 5)
);
PIDController rotationPID = new PIDController(
  1/90.0, 0, 0
  // new Constraints(swerveDrive.getMaximumChassisAngularVelocity(), 1080)
);

double travelTime = 1;
double distanceM = 1;
Pose2d sourcePose = new Pose2d();
double starttime = 0;
Field2d pidfield = new Field2d();
public Command pidToPoseInterpolated(Supplier<Pose2d> goalPose){
  SmartDashboard.putData("pidfield",pidfield);
  rotationPID.enableContinuousInput(-180, 180);

  Runnable onInit = ()->{
    sourcePose = swerveDrive.getPose();
    distanceM = sourcePose.getTranslation().getDistance(goalPose.get().getTranslation());
    travelTime = distanceM/swerveDrive.getMaximumChassisVelocity(); //doesn't handle accel
    pidfield.getObject("source").setPose(sourcePose);
    pidfield.getObject("goal").setPose(goalPose.get());
    SmartDashboard.putNumber("pidtest/traveltime", travelTime);
    xpid.reset();
    ypid.reset();
    rotationPID.reset();
    starttime = Timer.getFPGATimestamp();
  };
  Runnable onExecute = ()->{

    var ratio = (Timer.getFPGATimestamp()-starttime)/travelTime;
    ratio = MathUtil.clamp(ratio, 0, 1);
    SmartDashboard.putNumber("pidtest/ratio", ratio);

    var setPointPose = sourcePose.interpolate(goalPose.get(), ratio);
    xpid.setSetpoint(setPointPose.getX());
    ypid.setSetpoint(setPointPose.getY());
    rotationPID.setSetpoint(setPointPose.getRotation().getDegrees());

    //pid to pose math goes here
    var botpose = swerveDrive.getPose();
    double tx = xpid.calculate(botpose.getX());
    double ty = ypid.calculate(botpose.getY());
    double r = rotationPID.calculate(botpose.getRotation().getDegrees());

    SmartDashboard.putNumber("pidtest/xout", ratio);

    //set motor to use this
    setPrimaryInputsVoid(()->tx, ()->ty, ()->r);

    pidfield.getRobotObject().setPose(swerveDrive.getPose());
    pidfield.getObject("target").setPose(setPointPose);
  };
  Consumer<Boolean> onEnd = (cancelled)->{
    setPrimaryInputs(()->0.0, ()->0.0, ()->0.0);
  };
  BooleanSupplier isFinished = ()->{
      var dist = swerveDrive.getPose().getTranslation().getDistance(goalPose.get().getTranslation());
      var angle = rotationPID.getError();
      return dist < 0.2 && angle < 10;
      // return false;
  };

  // return new FunctionalCommand(onInit, onExecute, onEnd, isFinished, this);
  return new FunctionalCommand(onInit, onExecute, onEnd, isFinished, this).withTimeout(travelTime*4);
}




}
