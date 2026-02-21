// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.TargetingSystem;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.stormbots.LUT;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Subsystems.Swerve.Swerve;

public class TargetingSystem extends SubsystemBase {

  public static class ShooterState {
    public Angle turretAngle;
    public Angle hoodAngle;
    public double flywheelRPM;

    public Angle turretTolerance = Degrees.of(3);
    public Angle hoodTolerance = Degrees.of(3);
    public double flywheelTolerance = 300;

    public ShooterState(Angle turretAngle, Angle hoodAngle, double flywheelRPM){
      this.turretAngle = turretAngle;
      this.hoodAngle = hoodAngle;
      this.flywheelRPM = flywheelRPM;
    }

    public ShooterState withTurretTolerance(Angle tolerance){
      this.turretAngle = tolerance;
      return this;
    }

    public ShooterState withHoodTolerance(Angle tolerance){
      this.hoodAngle = tolerance;
      return this;
    }

    public ShooterState withFlywheelTolerance(double tolerance){
      this.flywheelTolerance = tolerance;
      return this;
    }
  }

  //distance, hoodangle, flywheel rpm
  LUT hubLUT = new LUT(new double[][]{
    {0, 0, 0}
  });

  //distance, hoodangle, flywheel rpm
  LUT passLUT = new LUT(new double[][]{
    {0, 0, 0}
  });

  Swerve swerve;

  Field2d field = new Field2d();

  /** Creates a new TargetingSubsystem. */
  public TargetingSystem(Swerve swerve){
    
    this.swerve = swerve;
    SmartDashboard.putData("targeting/field",field);

    
    field.getObject("testbot").setPose(new Pose2d(1,2,new Rotation2d()));
    field.getObject("pass").setPoses(
      new Pose2d(1,1.5,new Rotation2d()),
      new Pose2d(1,6.5,new Rotation2d())
      );
  }

  /** Figure out the best target based on conditions or field position */
  public Pose2d getBestTarget(Pose2d botPosition){
    var pass=field.getObject("pass").getPoses();
    if(botPosition.getX()<4.6) return new Pose2d(Constants.Field.blueHub,new Rotation2d());
    if(botPosition.getY()>=4) return pass.get(1);
    if(botPosition.getY()<4) return pass.get(0);
    return pass.get(0);
  }

  /** Generate a fieldcentric heading from bot location to target */
  public Rotation2d getHeadingToTarget(Pose2d botPose,Translation2d target){
    var translation=botPose.getTranslation();
    var angle = target.minus(translation).getAngle();
    return angle;
  }

  /** Return the distance between bot position and target */
  public Distance getDistanceToTarget(Pose2d botPose,Translation2d target){
    return Meters.of(botPose.getTranslation().minus(target).getNorm());
  }

  /** Generate a field-centric location of the turret's physical position. 
   * Mostly needed for sim and visualization.
   * */
  public Translation3d getTurretCenterpoint(){
    return new Translation3d(swerve.getSwervePose().getTranslation()).plus(Constants.Shooter.botToTurretOffset);
  }

  
  public ShooterState getShooterStateForHubTarget(Pose2d botPose, Translation2d target){
    Translation2d turretTranslation = botPose.getTranslation().plus(Constants.Shooter.botToTurretOffset.toTranslation2d());
    
    Distance magnitude = getDistanceToTarget(botPose, target);

    var hubOut = hubLUT.get(magnitude.in(Inches));   
    var angle = hubOut[1];
    var rpm = hubOut[2];

    return new ShooterState(Degrees.of(target.minus(turretTranslation).getAngle().getDegrees()), Degrees.of(angle), rpm);
  }

  @Override
  public void periodic() {
    var botpose = swerve.getSwervePose();
    field.setRobotPose(botpose);

    //Moving the Robot on the field is annoying, so sub in a draggable object for testing
    if(DriverStation.isDisabled()) botpose = field.getObject("testbot").getPose();

    var target = getBestTarget(botpose);
    var angle = getHeadingToTarget(botpose,target.getTranslation());
    var turret = new Pose2d(botpose.getX(),botpose.getY(),angle);

    field.getObject("turret").setPose(turret);
    field.getObject("bestTarget").setPose(getBestTarget(botpose));

    if(Robot.isSimulation()){
        //Generate a slightly fancier version for the 3D viewer
        var turret3d=new Pose3d(getTurretCenterpoint(),new Rotation3d(angle));
        DogLog.log("targeting/Turret", turret3d);
    }

  }

  @Override
  public void simulationPeriodic() {
  }



  /** Compute a Translation3d representing a velocity vector of a fired Fuel. 
   * Necessary for FuelSim testing.
   */
  public Translation3d simGenerateIdealShot(){
    var target = getBestTarget(swerve.getSwervePose()).getTranslation();
    var heading = getHeadingToTarget(swerve.getSwervePose(),target);
    var hoodangle=Degrees.of(60).in(Radians);

    return new Translation3d(
      InchesPerSecond.of(300).in(MetersPerSecond),
      new Rotation3d(0, hoodangle, heading.getRadians())
    );
  }



}
