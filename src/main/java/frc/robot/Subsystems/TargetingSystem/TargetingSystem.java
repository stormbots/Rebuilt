// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.TargetingSystem;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    public Angle hoodTolerance = Degrees.of(0.5);
    public double flywheelTolerance = 75;

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

  // distance, hoodangle, flywheel rpm
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
  public LUT passLUT = new LUT(new double[][]{
    {192+22, 45, 3200},

    {192+22+(7.5*12), 25, 4000},

    //min from center
    //max from midfield
    //from opponent alliance zone
  });

  Swerve swerve;

  //remove, we want to use field objects
  // ArrayList<Translation2d> bluePassTargets;
  Translation2d blueLow = new Translation2d(1,0.5);
  Translation2d blueHigh = new Translation2d(1,7.5);
  Translation2d redLow = new Translation2d(15.6,0.5);
  Translation2d redHigh = new Translation2d(15.6,7.5);

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
    // SmartDashboard.putNumber("bruh/bruh", value)
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

  
  public ShooterState getLUTShooterState(Supplier<Pose2d> botPose, Supplier<Translation2d> target, LUT lut){
    
    Distance magnitude = getDistanceToTarget(botPose.get().getTranslation(), target.get());

    
    SmartDashboard.putNumber("shooter/lut/distance", magnitude.in(Inches));
    var entry = lut.get(magnitude.in(Inches));   
    var angle = entry[1];
    var rpm = entry[2]+110;
    SmartDashboard.putNumber("shooter/lut/rpm", rpm);
    SmartDashboard.putNumber("shooter/lut/hoodangle", angle);
    
    Translation2d turretTranslation = getTurretCenterpoint().toTranslation2d();
    Rotation2d heading = getHeadingToTarget(turretTranslation, target.get());
    SmartDashboard.putNumber("shooter/turret/netHeading", heading.getDegrees());

    SmartDashboard.putNumber("shooter/turret/turretInput", heading.minus(botPose.get().getRotation()).getMeasure().in(Degrees));

    return new ShooterState(heading.minus(botPose.get().getRotation()).getMeasure(), Degrees.of(angle), rpm);
    // return new ShooterState(Degrees.of(0), Degrees.of(angle), rpm);
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

  public Translation2d getHubTarget(){
    return DriverStation.getAlliance().orElse(Alliance.Blue)==Alliance.Blue ?
      Constants.Field.blueHub :
      Constants.Field.redHub
    ;
  }

  public ShooterState getHub(){
    return getLUTShooterState(swerve::getSwervePose, this::getHubTarget, hubLUT);
  }


  public Angle getNearestAllianceWallAngle(Pose2d botPose){
    if( swerve.getSwervePose().getRotation().getMeasure().isNear(Degrees.of(90), Degrees.of(90)) ) return Degree.of(90);
    
    return Degrees.of(-90);
  }

  public Translation2d getPassTarget(){
    if(DriverStation.getAlliance().orElse(Alliance.Blue)==Alliance.Blue){
      return swerve.getSwervePose().getY() > 4.2 ? blueHigh : blueLow;
    }
    return swerve.getSwervePose().getY() > 4.2 ? redHigh : redLow;
  }

  public ShooterState getPass(){
    return getLUTShooterState(swerve::getSwervePose, this::getPassTarget, passLUT);
  }

  public ShooterState fixedShot(){
    return new ShooterState(Degrees.of(180), Degrees.of(10),2210 );
  }
  public ShooterState fixedPassNeutral(){
    return new ShooterState(Degrees.of(180), Degrees.of(45),3200 );
  }
  public ShooterState fixedPassOppAlliance(){
    return new ShooterState(Degrees.of(180), Degrees.of(25),4000 );
  }



}
