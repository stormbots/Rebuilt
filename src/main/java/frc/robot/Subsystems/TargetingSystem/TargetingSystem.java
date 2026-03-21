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

import java.util.function.Supplier;

import com.stormbots.LUT;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

    public Angle turretTolerance = Degrees.of(1.5);
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

  double timescalar = 1.1;
  double distanceoffset = 12.0;
  double fudgeFactor = 50.0;
  // distance, hoodangle, flywheel rpm
  LUT hubLUT = new LUT(new double[][]{
    // {22+14, 5, 2050},
    // {32+22, 10, 2050},
    // {48+22, 10, 2100},\\\\\\\\    // {60+22, 13, 2100},
    // {72+22, 23, 2100},
    // {84+22, 26, 2100},
    // {96+22, 30, 2150},
    // {108+22, 30, 2200},
    // {120+22, 30, 2300},
    // {132+22, 30, 2350},
    // {144+22, 30, 2400},
    // {156+22, 30, 2450},
    // {168+22, 30, 2500},
    // {180+22, 30, 2550},
    // {192+22, 30, 2625}
    // {22+14, 5, 2050, 0.8},
    // {32+22, 10, 2050, 0.8},
    // {48+22, 10, 2100, 0.8},
    // {60+22, 13, 2100, 0.8},
    // {72+22, 23, 2100, 0.8},
    // {84+22, 26, 2100, 0.8},
    // {96+22, 30, 2150, 0.5763},
    // {108+22, 30, 2200, 0.6094},
    // {120+22, 30, 2300, 0.64250},
    // {132+22, 30, 2350, 0.67559},
    // {144+22, 30, 2400, 0.70869},
    // {156+22, 30, 2450, 0.74179},
    // {168+22, 30, 2500, 0.77489},
    // {180+22, 30, 2550, 0.80798},
    // {192+22, 30, 2625, 0.84108},
    // {3*12+24, 12, 2245, 0.5},
    // {4*12+24, 12, 2255, 0.5},
    // {5*12+24, 13.5, 2250, 0.5},
    // {6*12+24, 17, 2270, 0.5},
    // {7*12+24, 20, 2355, 0.5},
    // {8*12+24, 23, 2425, 0.5763},
    // {9*12+24, 23, 2475, 0.6094},
    // {10*12+24, 23, 2510, 0.64250},
    // {11*12+24, 23, 2550, 0.67559},
    // {12*12+24, 23, 2600, 0.70869},
    // {13*12+24, 23, 2625, 0.74179},
    // {14*12+24, 23, 2712.5, 0.77489},
    // {15*12+24, 23, 2775, 0.80798},
    // {16*12+24, 23, 2825, 0.84108},
    //EVERYTHING UNDER THIS IS PROBABLY ACTUALLY CORRECT
    {3*12+24+distanceoffset, 12, 2245+fudgeFactor, 0.417*timescalar},
    {4*12+24+distanceoffset, 12, 2255+fudgeFactor, 0.467*timescalar},
    {5*12+24+distanceoffset, 13.5, 2250+fudgeFactor, 0.517*timescalar},
    {6*12+24+distanceoffset, 17, 2270+fudgeFactor, 0.5*timescalar},
    {7*12+24+distanceoffset, 20, 2355+fudgeFactor, 0.467*timescalar},
    {8*12+24+distanceoffset, 23, 2425+fudgeFactor, 0.500*timescalar},
    {9*12+24+distanceoffset, 23, 2475+fudgeFactor, 0.483*timescalar},
    {10*12+24+distanceoffset, 23, 2510+fudgeFactor, 0.500*timescalar},
    {11*12+24+distanceoffset, 23, 2550+fudgeFactor, 0.550*timescalar},//EVERY TOF BELOW THIS NEEDS TO BE DOUBLE CHECKED
    {12*12+24+distanceoffset, 23, 2600+fudgeFactor, 0.6*timescalar},
    {13*12+24+distanceoffset, 23, 2625+fudgeFactor, 0.63*timescalar},
    {14*12+24+distanceoffset, 23, 2712.5+fudgeFactor+15, 0.7*timescalar},
    {15*12+24+distanceoffset, 23, 2775+fudgeFactor+15, 0.75*timescalar},
    {16*12+24+distanceoffset, 23, 2850+fudgeFactor+15, 0.8*timescalar} 

  });
  

  //distance, hoodangle, flywheel rpm
  public LUT passLUT = new LUT(new double[][]{
    {169, 25, 2000, 1.2},
    {225, 35, 2500, 1.2},
    {32*12, 35, 3000, 1.2},
    {40*12, 40, 3500, 1.5},
    {47*12, 40, 3600, 1.5}

    //min from center
    //max from midfield
    //from opponent alliance zone
  });

  Swerve swerve;

  //remove, we want to use field objects
  // ArrayList<Translation2d> bluePassTargets;
  Translation2d blueLow = new Translation2d(1,1.5);
  Translation2d blueHigh = new Translation2d(1,6.5);
  Translation2d redLow = new Translation2d(15.6,1.5);
  Translation2d redHigh = new Translation2d(15.6,6.5);

  Field2d field = new Field2d();

  /** Creates a new TargetingSubsystem. */
  public TargetingSystem(Swerve swerve){
    
    this.swerve = swerve;
    SmartDashboard.putData("targeting/field",field);

    field.getObject("testbot").setPose(new Pose2d(1,2,new Rotation2d()));

    field.getObject("pass").setPoses(
      new Pose2d(blueLow,new Rotation2d()),
      new Pose2d(blueHigh,new Rotation2d()),
      new Pose2d(redLow,new Rotation2d()),
      new Pose2d(redHigh,new Rotation2d())
    );
  }

  /** Figure out the best target based on conditions or field position */
  public Pose2d getBestTarget(Pose2d botPosition){
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    if(alliance==Alliance.Blue){
      if(botPosition.getX()<4.6) return new Pose2d(getHubTarget(),new Rotation2d());
    }else{
      if(botPosition.getX()>11.9) return new Pose2d(getHubTarget(),new Rotation2d());
    }
    return new Pose2d(getPassTarget(),new Rotation2d());
  }

  public Pose2d getTarLockTemp(){
    return new Pose2d(Constants.Field.blueHub,new Rotation2d());
  }

  private Pose2d getBestTarget(){
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
    var rpm = entry[2];
    SmartDashboard.putNumber("shooter/lut/rpm", rpm);
    SmartDashboard.putNumber("shooter/lut/hoodangle", angle);
    
    Translation2d turretTranslation = getTurretCenterpoint().toTranslation2d();
    Rotation2d heading = getHeadingToTarget(turretTranslation, target.get());
    SmartDashboard.putNumber("shooter/turret/netHeading", heading.getDegrees());

    SmartDashboard.putNumber("shooter/turret/turretInput", heading.minus(botPose.get().getRotation()).getMeasure().in(Degrees));

    return new ShooterState(heading.minus(botPose.get().getRotation()).getMeasure(), Degrees.of(angle), rpm);
    // return new ShooterState(Degrees.of(0), Degrees.of(angle), rpm);
  }

  public Translation2d getBotVelCompensatedTarget(Supplier<Pose2d> botPose, Supplier<Translation2d> target, LUT lut, Supplier<ChassisSpeeds> botVelocity){
    Distance magnitude = getDistanceToTarget(botPose.get().getTranslation(), target.get());
    
    var entry = lut.get(magnitude.in(Inches));   
    var tof = entry[3];

    Translation2d botVelocityTranslation = new Translation2d(botVelocity.get().vxMetersPerSecond, botVelocity.get().vyMetersPerSecond);

    //Bot velocity * Time of Flight = how much impact in the unit of distance the bots velocity will have on the shot
    //Since we want to compensate for this, find the inverse of this vector and apply to our target
    Translation2d distanceCompensation = botVelocityTranslation.times(tof)
    .unaryMinus();
    
    //This is where we would have to aim, assuming we are static, to compensate
    //Hence, call it virtual target, as it is not our "true" target, but is effectively what is known to the shooter
    Translation2d virtualTarget = target.get().plus(distanceCompensation);

    //However, with a changed target, our shot trajectory changes
    //Hence, we will have a changed time of flight
    //Repeat the above process until the change between each iteration is negligible
    //Essentially, the virtual target stabilizes
    //TODO: change from a static amount of 3 iterations to dynamically ensuring percent change is negligible (eg. 2% or less)
    for(int i=0; i<5; i++){
      magnitude = getDistanceToTarget(botPose.get().getTranslation(), virtualTarget);

      entry = lut.get(magnitude.in(Inches));
      tof = entry[3]; 

      distanceCompensation = botVelocityTranslation.times(tof).unaryMinus(); 

      //new distance compensation is always applied to TARGET not VIRTUALTARGET
      //This is since the goal of each iteration is to get a tof that approaches the tof of ideal shot
      virtualTarget = target.get().plus(distanceCompensation);
    }

    return virtualTarget;
  }

  //Compensates for drivetrain velocity
  public ShooterState getLUTShooterStateBotVelCompensated(Supplier<Pose2d> botPose, Supplier<Translation2d> target, LUT lut, Supplier<ChassisSpeeds> botVelocity){
    Supplier<Translation2d> virtualTarget = ()->getBotVelCompensatedTarget(botPose, target, lut, botVelocity);

    return getLUTShooterState(botPose, virtualTarget, lut);
  }

  @Override
  public void periodic() {
    var botpose = swerve.getSwervePose();
    field.setRobotPose(botpose);

    //Moving the Robot on the field is annoying, so sub in a draggable object for testing
    //Key functions don't accept poses and read only swerve pose; Disabling this to avoid confusion
    // if(DriverStation.isDisabled()) botpose = field.getObject("testbot").getPose();


    var target = getBestTarget(botpose);
    var angle = getHeadingToTarget(botpose.getTranslation(),target.getTranslation());
    var turret = new Pose2d(botpose.getX(),botpose.getY(),angle);

    var bestTarget = getBestTarget(botpose);

    field.getObject("turret").setPose(turret);
    field.getObject("bestTarget").setPose(bestTarget);

    // field.getObject("bestTargetCompensated").setPose(compen);

    if(Robot.isSimulation()){
        //Generate a slightly fancier version for the 3D viewer
        var turret3d=new Pose3d(getTurretCenterpoint(),new Rotation3d(angle));
        DogLog.log("targeting/Turret", turret3d);
    }

    var compensatedTarget = getBotVelCompensatedTarget(
      swerve::getSwervePose,
      bestTarget::getTranslation,
      hubLUT,
      swerve::getChassisSpeedsFieldRelative
    );

    field.getObject("bestTargetCompensated").setPose(new Pose2d(compensatedTarget,new Rotation2d()));

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

  public ShooterState getHubBotVelCompensated(){
    return getLUTShooterStateBotVelCompensated(swerve::getSwervePose, this::getHubTarget, hubLUT, swerve::getFieldRelativeChassisSpeeds);
  }

  //IDT this is needed for now. We'll see. if it is, i'd like getPass() to use this method
  // public ShooterState getGroundShot(Pose2d target){
  //   return getGroundShooterState(swerve.getSwervePose(),target.getTranslation(), passLUT);
  // }


  // private Translation2d getClosest(Translation2d bot, Collection<Translation2d> targets){
  //   // ArrayList<Translation2d>.of(new Translation2d(),new Translation2d());
  //   new Arraylist {new Translation2d(),new Translation2d()};
  //   return bot.nearest(aaaaa);
  // }

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

    // Translation2d target = new Translation2d(2, 2); //get best target
    // return getLUTShooterState(swerve::getSwervePose,()->target, passLUT);

    // Translation2d turretTranslation = getTurretCenterpoint().toTranslation2d();
    // Rotation2d heading = getHeadingToTarget(turretTranslation, getPassTarget());

    // return new ShooterState(heading.minus(swerve.getSwervePose().getRotation()).getMeasure(), Degrees.of(30), 2760);
  }

  public ShooterState getPassBotVelCompensated(){
    return getLUTShooterStateBotVelCompensated(swerve::getSwervePose, this::getPassTarget, passLUT, swerve::getFieldRelativeChassisSpeeds);
  }

  public ShooterState fixedShot(){
    return new ShooterState(Degrees.of(180), Degrees.of(10),2210 );
  }
  public ShooterState fixedPassNeutral(){
    return new ShooterState(Degrees.of(180), Degrees.of(45),3200 );
  }
  public ShooterState fixedPassOppAlliance(){
    return new ShooterState(Degrees.of(180), Degrees.of(40),3600 );
  }



}
