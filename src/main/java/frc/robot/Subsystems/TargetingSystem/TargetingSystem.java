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
    {22+14, 5, 2050},
    {32+22, 10, 2050},
    {48+22, 10, 2100},
    {60+22, 13, 2100},
    {72+22, 23, 2100},
    {84+22, 26, 2100},
    {96+22, 30, 2150},
    {108+22, 30, 2200},
    {120+22, 30, 2300},
    {132+22, 30, 2350},
    {144+22, 30, 2400},
    {156+22, 30, 2450},
    {168+22, 30, 2500},
    {180+22, 30, 2550},
    {192+22, 30, 2625}
  });

  //distance, hoodangle, flywheel rpm
  LUT passLUT = new LUT(new double[][]{
    {0, 0, 0},
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
  private Pose2d getBestTarget(Pose2d botPosition){
    var pass=field.getObject("pass").getPoses();
    if(botPosition.getX()<4.6) return new Pose2d(Constants.Field.blueHub,new Rotation2d());
    if(botPosition.getY()>=4) return pass.get(1);
    if(botPosition.getY()<4) return pass.get(0);
    return pass.get(0);
  }

  public Pose2d getTarLockTemp(){
    return new Pose2d(Constants.Field.blueHub,new Rotation2d());
  }

  public Pose2d getBestTarget(){
    return getBestTarget(swerve.getSwervePose());
  }

  /** Generate a fieldcentric heading from bot location to target */
  public Rotation2d getHeadingToTarget(Translation2d botTranslation,Translation2d target){
    var angle = target.minus(botTranslation).getAngle();
    return angle;
  }

  /** Return the distance between bot position and target */
  public Distance getDistanceToTarget(Translation2d botTranslation,Translation2d target){
    return Meters.of(botTranslation.minus(target).getNorm());
  }

  /** Generate a field-centric location of the turret's physical position. 
   * */
  public Translation3d getTurretCenterpoint(){
    return new Translation3d(swerve.getSwervePose().getTranslation()).plus(Constants.Shooter.botToTurretOffset);
  }

  
  private ShooterState getLUTShooterState(Pose2d botPose, Translation2d target, LUT lut){
    
    Distance magnitude = getDistanceToTarget(botPose.getTranslation(), target);

    
    SmartDashboard.putNumber("shooter/lut/botx", botPose.getX());
    SmartDashboard.putNumber("shooter/lut/boty", botPose.getY());
    SmartDashboard.putNumber("shooter/lut/targetx", target.getX());
    SmartDashboard.putNumber("shooter/lut/targety", target.getY());
    SmartDashboard.putNumber("shooter/lut/distance", magnitude.in(Inches));
    var entry = lut.get(magnitude.in(Inches));   
    var angle = entry[1];
    var rpm = entry[2];
    SmartDashboard.putNumber("shooter/lut/rpm", rpm);
    SmartDashboard.putNumber("shooter/lut/hoodangle", angle);
    
    Translation2d turretTranslation = getTurretCenterpoint().toTranslation2d();
    Rotation2d heading = getHeadingToTarget(turretTranslation, target);

    // return new ShooterState(heading.minus(botPose.getRotation()).getMeasure(), Degrees.of(angle), rpm);
    return new ShooterState(Degrees.of(0), Degrees.of(angle), rpm);
  }


  @Override
  public void periodic() {
    var botpose = swerve.getSwervePose();
    field.setRobotPose(botpose);

    //Moving the Robot on the field is annoying, so sub in a draggable object for testing
    if(DriverStation.isDisabled()) botpose = field.getObject("testbot").getPose();

    var target = getBestTarget(botpose);
    var angle = getHeadingToTarget(botpose.getTranslation(),target.getTranslation());
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
    var heading = getHeadingToTarget(swerve.getSwervePose().getTranslation(),target);
    var hoodangle=Degrees.of(60).in(Radians);

    return new Translation3d(
      InchesPerSecond.of(300).in(MetersPerSecond),
      new Rotation3d(0, hoodangle, heading.getRadians())
    );
  }

  public ShooterState getHub(){
    Translation2d target = new Translation2d(); //get best target
    //TODO: NEED TO CHANGE TO FLIP BASED OFF FIELD
    return getLUTShooterState(swerve.getSwervePose(), Constants.Field.blueHub, hubLUT);
  }

  //IDT this is needed for now. We'll see. if it is, i'd like getPass() to use this method
  // public ShooterState getGroundShot(Pose2d target){
  //   return getGroundShooterState(swerve.getSwervePose(),target.getTranslation(), passLUT);
  // }

  public ShooterState getPass(){
    Translation2d target = new Translation2d(); //get best target
    return getLUTShooterState(swerve.getSwervePose(),target, passLUT);
  }



}
