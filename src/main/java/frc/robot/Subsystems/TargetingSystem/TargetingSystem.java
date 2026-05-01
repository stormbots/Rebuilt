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
  //NEW VALUES HOPEFULLY HELP SMOOTH SOTM
  private Translation2d smoothedBotVelocity = new Translation2d();
  private static final double VELOCITY_ALPHA = 0.15; // lower = more smoothing (tune this)
  private static final double CONVERGENCE_THRESHOLD_METERS = 0.005; // 5mm
  public static class ShooterState {
  
    public Angle turretAngle;
    public Angle hoodAngle;
    public double flywheelRPM;

    public Angle turretTolerance = Degrees.of(1.5);
    public Angle hoodTolerance = Degrees.of(0.5);
    public double flywheelTolerance = 250;

    public ShooterState(Angle turretAngle, Angle hoodAngle, double flywheelRPM){
      this.turretAngle = turretAngle;
      this.hoodAngle = hoodAngle;
      this.flywheelRPM = flywheelRPM;
    }

    public ShooterState withTurretTolerance(Angle tolerance){
      this.turretTolerance = tolerance;
      return this;
    }

    public ShooterState withHoodTolerance(Angle tolerance){
      this.hoodTolerance = tolerance;
      return this;
    }

    public ShooterState withFlywheelTolerance(double tolerance){
      this.flywheelTolerance = tolerance;
      return this;
    }
  }

  double fps=240.0;
  // double timescalar = 1.0;
  // double distanceoffset = 0.0;
  double distFactor = 15.0;
  // double distFactor = 0.0;

  // distance, hoodangle, flywheel rpm, TOF
  LUT hubLUT = new LUT(new double[][]{
    //EVERYTHING UNDER THIS IS PROBABLY ACTUALLY CORRECT
    // {3*12+24+distanceoffset, 12, 2245+fudgeFactor, 0.417*timescalar},
    // {4*12+24+distanceoffset, 12, 2255+fudgeFactor, 0.467*timescalar},
    // {5*12+24+distanceoffset, 13.5, 2250+fudgeFactor, 0.517*timescalar},
    // {6*12+24+distanceoffset, 17, 2270+fudgeFactor, 0.5*timescalar},
    // {7*12+24+distanceoffset, 20, 2355+fudgeFactor, 0.467*timescalar},
    // {8*12+24+distanceoffset, 23, 2425+fudgeFactor, 0.500*timescalar},
    // {9*12+24+distanceoffset, 23, 2475+fudgeFactor, 0.483*timescalar},
    // {10*12+24+distanceoffset, 23, 2510+fudgeFactor, 0.500*timescalar},
    // {11*12+24+distanceoffset, 23, 2550+fudgeFactor, 0.550*timescalar},//EVERY TOF BELOW THIS NEEDS TO BE DOUBLE CHECKED
    // {12*12+24+distanceoffset, 23, 2600+fudgeFactor, 0.6*timescalar},
    // {13*12+24+distanceoffset, 23, 2625+fudgeFactor, 0.63*timescalar},
    // {14*12+24+distanceoffset, 23, 2712.5+fudgeFactor+15, 0.7*timescalar},
    // {15*12+24+distanceoffset, 23, 2775+fudgeFactor+15, 0.75*timescalar},
    // {16*12+24+distanceoffset, 23, 2850+fudgeFactor+15, 0.8*timescalar}

    //Quinn doesn't seem to want to delete things so adding new lines
    { 41,  5, 2045, 129/fps },
    { 41+distFactor,  5, 2245, 129/fps },
    { 60+distFactor, 10, 2245, 127/fps },
    { 84+distFactor, 21, 2295, 114/fps },
    {108+distFactor, 23, 2320, 117/fps },
    //NEED TO RETIME THIS ONE CUZ THE SHOT IS BAD
    {132+distFactor, 23, 2470, 129/fps },
    {156+distFactor, 23, 2540, 135/fps },
    {180+distFactor, 25, 2670, 142/fps },
    {204+distFactor, 25, 2770, 156/fps },
    {216+distFactor, 25, 2800, 160/fps }, //observed TOF: +/-3 frames
    {230+distFactor, 25, 2875, 160/fps }

    //far corner
  });

  //distance, hoodangle, flywheel rpm
  public LUT passLUT = new LUT(new double[][]{
    {169, 25, 2000, 138/fps},
    {225, 35, 2500, 152/fps},
    {32*12, 35, 3000, 184/fps},
    {40*12, 40, 3500, 193/fps},
    {47*12, 40, 3700, 219/fps}

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

    if(alliance == Alliance.Blue && botPosition.getX() < 4.6){
      return new Pose2d(getHubTarget(), new Rotation2d());
    }
    
    if(alliance != Alliance.Blue && botPosition.getX() > 11.9){
      return new Pose2d(getHubTarget(), new Rotation2d());
    }

    return new Pose2d(getPassTarget(), new Rotation2d());
  }

  public LUT getBestLutTarget(Pose2d botPosition){
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    if(alliance == Alliance.Blue && botPosition.getX() < 4.6){
      return hubLUT;
    }
    
    if(alliance != Alliance.Blue && botPosition.getX() > 11.9){
      return hubLUT;
    }

    return passLUT;
  }

  public Pose2d getTarLockTemp(){
    return new Pose2d(Constants.Field.blueHub, new Rotation2d());
  }

  /** Generate a fieldcentric heading from bot location to target */
  public Rotation2d getHeadingToTarget(Translation2d botTranslation,Translation2d target){
    return target.minus(botTranslation).getAngle();
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
  }
 
  //Velocity compensation with jitter filtering, and average moving velocity smoothing
  public Translation2d getBotVelCompensatedTarget(
    Supplier<Pose2d> botPose, Supplier<Translation2d> target, LUT lut, Supplier<ChassisSpeeds> botVelocity){
    // Smooth velocity with exponential moving average
    // Call .get() once and store — avoids double-sampling across the loop
    //What I HAVE done in the past is multiply these values by a factor if it looks like one is being affected more or less,
    //that is difficult to determine cuz it could just be that turret is too slow or something 
    //but we MIGHT want to do something similar in the future if it looks like we are missing more forwards/backwards or sideways
    ChassisSpeeds speeds = botVelocity.get();
    Translation2d rawVelocity = new Translation2d(
        speeds.vxMetersPerSecond,
        speeds.vyMetersPerSecond
    );
    smoothedBotVelocity = smoothedBotVelocity.interpolate(rawVelocity, VELOCITY_ALPHA);

    // Optionally dead-band tiny velocities that are just noise
    double speed = smoothedBotVelocity.getNorm();
    Translation2d effectiveVelocity = speed < 0.05 // m/s threshold, we go pretty slow on sotm so this needs to be more precise than you would think
        ? new Translation2d()
        : smoothedBotVelocity;

    Translation2d currentTarget = target.get();
    Translation2d botTranslation = botPose.get().getTranslation();

    // Initial compensation
    Distance magnitude = getDistanceToTarget(botTranslation, currentTarget);
    double tof = lut.get(magnitude.in(Inches))[3];
    Translation2d virtualTarget = currentTarget.plus(effectiveVelocity.times(tof).unaryMinus());

    for (int i = 0; i < 10; i++) {
        magnitude = getDistanceToTarget(botTranslation, virtualTarget);
        tof = lut.get(magnitude.in(Inches))[3];
        Translation2d newVirtualTarget = currentTarget.plus(effectiveVelocity.times(tof).unaryMinus());

        // Break early when change is negligible
        if (newVirtualTarget.getDistance(virtualTarget) < CONVERGENCE_THRESHOLD_METERS) {
            virtualTarget = newVirtualTarget;
            break;
        }
        virtualTarget = newVirtualTarget;
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
    var bestLut = getBestLutTarget(botpose);

    field.getObject("turret").setPose(turret);
    field.getObject("bestTarget").setPose(bestTarget);

    // field.getObject("bestTargetCompensated").setPose(compen);

    if(Robot.isSimulation()){
        //Generate a slightly fancier version for the 3D viewer
        var turret3d = new Pose3d(getTurretCenterpoint(),new Rotation3d(angle));
        DogLog.log("targeting/Turret", turret3d);
    }

    var compensatedTarget = getBotVelCompensatedTarget(
      swerve::getSwervePose,
      bestTarget::getTranslation,
      bestLut,
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

  public Angle getNearestAllianceWallAngle(Pose2d botPose){
    if(swerve.getSwervePose().getRotation().getMeasure().isNear(Degrees.of(90), Degrees.of(90))) {
      return Degree.of(90);
    }
    
    return Degrees.of(-90);
  }

  public Translation2d getPassTarget(){
    if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue){
      return swerve.getSwervePose().getY() > 4.2 ? blueHigh : blueLow;
    }
    return swerve.getSwervePose().getY() > 4.2 ? redHigh : redLow;
  }

  public ShooterState getPass(){
    return getLUTShooterState(swerve::getSwervePose, this::getPassTarget, passLUT);
  }

  public ShooterState getPassBotVelCompensated(){
    return getLUTShooterStateBotVelCompensated(swerve::getSwervePose, this::getPassTarget, passLUT, swerve::getFieldRelativeChassisSpeeds);
  }

  //idle the hood and flywheels
  public ShooterState getTurretTracking(){
    boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;

    //if we're within the shooting zones for our respective alliance
    if(
      isBlue && swerve.getSwervePose().getX() < 4.572 ||
      !isBlue && swerve.getSwervePose().getX() > 16.535-4.572
    ){
      return getHubBotVelCompensated();
    }
    
    return getPassBotVelCompensated();
  }

  public ShooterState fixedShot(){
    return new ShooterState(Degrees.of(-180), Degrees.of(10),2210 );
  }
  public ShooterState fixedPassNeutral(){
    return new ShooterState(Degrees.of(-180), Degrees.of(45),3200 );
  }
  public ShooterState fixedPassOppAlliance(){
    return new ShooterState(Degrees.of(-180), Degrees.of(40),3600 );
  }
}
