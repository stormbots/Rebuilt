// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Climber.Climber;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Questnav.QuestNavSubsystem;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Spindexer.Spindexer;
import frc.robot.Subsystems.Swerve.Pathing;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.TargetingSystem.TargetingSystem;
import frc.robot.lib.BLine.FlippingUtil;
import frc.robot.lib.BLine.Path;


/** Add your docs here. */
public class Autos {
  Swerve swerve;
  Shooter shooter;
  Intake intake;
  QuestNavSubsystem questNav;
  Spindexer spindexer;
  Pathing pathing;
  TargetingSystem targeting;
  Climber climber;

  SendableChooser<Supplier<Command>> autoChooser = new SendableChooser<>();
  // private CompletableFuture<Command> selectedAutoFuture =
  // CompletableFuture.supplyAsync(()->new InstantCommand());

  // EVIL trenchPoses, IN IS OUR STARTING TRENCH POSES
  Pose2d preTrenchInBlueLeft = new Pose2d(4.0, 7.4, new Rotation2d());
  Pose2d preTrenchInBlueRight = new Pose2d(4.0, 0.6, new Rotation2d());
  Pose2d preTrenchOutBlueRight = new Pose2d(6.0, 0.6, new Rotation2d(Degrees.of(180)));
  Pose2d preTrenchOutBlueLeft = new Pose2d(6.0, 7.4, new Rotation2d(Degrees.of(180)));

  // EVIL HAlFWAY POSES
  Pose2d neutralHalfWayRight = new Pose2d(8.6, 0.6, new Rotation2d(Degrees.of(120)));
  Pose2d neutralHalfwayLeft = new Pose2d(); // TODO:Flip ts

  // EVIL PRE INTAKE POSES
  Pose2d neutralPreIntRight = new Pose2d(8.6, 1.2, new Rotation2d(Degrees.of(120)));
  Pose2d neutralPreIntLeft = new Pose2d(8.6, 1.2, new Rotation2d(Degrees.of(120))); // TODO: Flip TS

  // EVIL INTAKING POSE
  Pose2d neutralIntRight = new Pose2d(8.6, 3.5, new Rotation2d(Degrees.of(120)));
  Pose2d neutralIntLeft = new Pose2d(8.6, 3.5, new Rotation2d(Degrees.of(120))); // TODO: Flip TS

  // EVIL SCOOPER THINGY
  Pose2d scoopFuel = new Pose2d(7.64, 0.88, new Rotation2d(Degrees.of(120)));

  // SORT OF THE BEST SHOOTING POSE
  Pose2d shotPoseBL = new Pose2d(3.6, 7.4, new Rotation2d(Degrees.of(180)));
  Pose2d shotPoseRight = new Pose2d(3.6, 0.6, new Rotation2d(Degrees.of(180)));

  // Depot poses
  Pose2d preIntDepot = new Pose2d(1.0, 6.0, new Rotation2d(Degrees.of(180)));
  Pose2d preIntDepotRed = new Pose2d(15.5, 2.0, new Rotation2d());
  Pose2d intDepot = new Pose2d(0.52, 6.0, new Rotation2d(Degrees.of(180 - 15)));
  Pose2d intDepotRed = new Pose2d(16.02, 2.0, new Rotation2d(Degrees.of(15)));

  // FORCED POSES
  Pose2d startRedLeft = new Pose2d(12.577 + 0.37465, 0.4445, new Rotation2d(Degrees.of(-180)));
  Pose2d startRedRight = new Pose2d(12.577 + 0.37465, 7.6755, new Rotation2d(Degrees.of(-180)));
  Pose2d startBlueLeft = new Pose2d(3.52535 + 0.8, 7.6755, new Rotation2d());
  Pose2d startBlueRight = new Pose2d(3.52535 + 0.8, 0.4445, new Rotation2d());

  // Commonly used paths
  Path centerShootPath = new Path("CenterShootAutoV2");
  Path shootIntitialPath = new Path("shootInitial");
  Path postShootToClimb = new Path("postShootingToClimb");
  Path climbAutoRedSide = new Path("climbAuto");

  public Autos(
      Swerve swerve,
      Shooter shooter,
      Intake intake,
      QuestNavSubsystem questNav,
      Spindexer spindexer,
      Pathing pathing,
      TargetingSystem targeting,
      Climber climber) {
    this.swerve = swerve;
    this.shooter = shooter;
    this.intake = intake;
    this.questNav = questNav;
    this.spindexer = spindexer;
    this.pathing = pathing;
    this.targeting = targeting;
    this.climber = climber;

    
    // FORCE SET POSE IS CHOPPED DO NOT USE, shouldn't even be on their dashboard,
    // needs fixing and could still prove useful
    SmartDashboard.putBoolean("ForceSetPose", SmartDashboard.getBoolean("ForceSetPose", false));
    SmartDashboard.setPersistent("ForceSetPose");

    // ACTUAL OPTIONS BELOW HERE
    autoChooser.addOption("Red Depot", this::redDepot);
    autoChooser.addOption("Blue Depot", this::blueDepot);
    autoChooser.addOption("ShootOnlyEight", this::basicShootToEmpty);

    autoChooser.addOption("RL Center Auto", this::CenterShootAutoRedLEFT);
    autoChooser.addOption("BL Center Auto", this::CenterShootAutoBlueLEFT);
    autoChooser.addOption("RR  Center Auto", this::CenterShootAutoRedRIGHT);
    autoChooser.addOption("BR Center Auto", this::CenterShootAutoBlueRIGHT);

    autoChooser.addOption("RL Passing Auto", this::PassingAutoRedLEFT);
    autoChooser.addOption("BL Passing Auto", this::PassingAutoBlueLEFT);
    autoChooser.addOption("RR Passing Auto", this::PassingAutoRedRIGHT);
    autoChooser.addOption("BR Passing Auto", this::PassingAutoBlueRIGHT);

    autoChooser.setDefaultOption("Select Auto", () -> new InstantCommand());
    autoChooser.addOption("VV UNTESTED VV", () -> new InstantCommand());
    autoChooser.addOption("ClimbAutoChoppedWhyAreWeDoingThisIWannaShootSoBad", this::climbAutoRedLeft); // best name ever
    SmartDashboard.putData("AutoSelector/chooser", autoChooser);
  }

  // Get Auto Command
  public Command getAutonomousCommand() {
    return autoChooser.getSelected().get();
  }

  private Pose2d autoTeamFlippedPose(double x, double y, double degrees) {
    var pose = new Pose2d(x, y, new Rotation2d(Degree.of(degrees)));
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red)
      pose = FlippingUtil.flipFieldPose(pose);
    return pose;
  }

  /////////////////////////
  // ALL AUTOS BELLOW HERE sk was here!!!!!!!!!//
  /////////////////////////

  // BASIC STUFF, DO NOT RUN AT THE START OF A BLINE AUTO, IT BREAKS EVENT
  // TRIGGERS AND MAKES PATH CHOPPED, SUPER LAST RESORT AUTOS
  public Command basicShootInitial8() {
    return Commands.sequence(
      //   return targeting.getHeadingToTarget(swerve.getSwervePose().getTranslation(), targeting.getHubTarget())
      //     .plus(Rotation2d.k180deg);
      // }).until(() -> swerve.isOnTargetAngle()).withTimeout(1.0),
      shootAuto().withTimeout(1.5)
    );
  }

  public Command basicShootToEmpty() {
    return Commands.sequence(
      swerve.turnToHeadingWithinTurretRange(() -> {
        return targeting.getHeadingToTarget(swerve.getSwervePose().getTranslation(), targeting.getHubTarget())
          .plus(Rotation2d.k180deg);
      }).until(() -> swerve.isOnTargetAngle()).withTimeout(1.0),
      shootAuto().withTimeout(5)
    );
  }

  // THESE ARE PID AUTOS NOT BLINE, idk where we're gonna be at after today's
  // practice so I will leave just this one for future
  public Command depotAutoBlue() {
    return Commands.sequence(
      basicShootInitial8(),
      swerve.pidToPose(() -> preIntDepot).alongWith(intake.intake()).until(() -> swerve.isOnTargetTranslate()),
      driveToPose(intDepot, intakeWhileShooting()),
      intakeWhileShooting()
    );
  }

  public Command depotAutoRed() {
    return Commands.sequence(
      basicShootInitial8(),
      driveToPose(preIntDepot, stow()),
      driveToPose(intDepot, intakeWhileShooting()),
      intakeWhileShooting()
    );
  }

  // ALL OF THESE AUTOS ARE THE GOOD BLINE TYPE

  public Command CenterShootAutoRedLEFT() {
    return Commands.sequence(
      basicShootInitial8(),
      pathing.followPathTeamFlipped(centerShootPath).withTimeout(12),
      pathing.followPathTeamFlipped(postShootToClimb).withTimeout(2.0),
      climbAutoRedLeft()
    );
  }

  public Command CenterShootAutoRedRIGHT() {
    shootIntitialPath.mirror();
    centerShootPath.mirror();
    postShootToClimb.mirror();

    return Commands.sequence(
      basicShootInitial8(),
      pathing.followPathTeamFlipped(centerShootPath).withTimeout(12),
      pathing.followPathTeamFlipped(postShootToClimb).withTimeout(2.0),
      climbAutoRedRight()
    );
  }

  public Command CenterShootAutoBlueLEFT() {
    return Commands.sequence(
      basicShootInitial8(),
      pathing.followPath(centerShootPath).withTimeout(12),
      pathing.followPath(postShootToClimb).withTimeout(2.0),
      climbAutoBlueLeft()
    );
  }

  public Command CenterShootAutoBlueRIGHT() { 
    shootIntitialPath.mirror();
    centerShootPath.mirror();
    return Commands.sequence(
      pathing.followPath(shootIntitialPath).withTimeout(2.0),
      pathing.followPath(centerShootPath).withTimeout(20),
      pathing.followPath(centerShootPath).withTimeout(20)
    );
  }

  public Command PassingAutoRedLEFT() {
    return Commands.sequence(
      pathing.followPathTeamFlipped(shootIntitialPath).withTimeout(3.0),
      pathing.followPathTeamFlipped(new Path("testingStraightUnder")).withTimeout(20),
      pathing.followPathTeamFlipped(new Path("testingStraightUnder")).withTimeout(20)
    );
  }

  public Command PassingAutoRedRIGHT() {
    shootIntitialPath.mirror();
    Path path = new Path("testingStraightUnder");
    path.mirror();
    return Commands.sequence(
      pathing.followPathTeamFlipped(shootIntitialPath).withTimeout(3.0),
      pathing.followPathTeamFlipped(path).withTimeout(20),
      pathing.followPathTeamFlipped(path).withTimeout(20)
    );
  }

  public Command PassingAutoBlueLEFT() {
    return Commands.sequence(
      pathing.followPath(shootIntitialPath).withTimeout(3.0),
      pathing.followPath(new Path("testingStraightUnder")),
      pathing.followPath(new Path("testingStraightUnder"))
    );
  }

  public Command climbAutoRedLeft() {
    return Commands.sequence(
      pathing.followPath(climbAutoRedSide).withTimeout(5.0).until(()->climber.rangefinders.isDetectable()),
      new ParallelCommandGroup(
        swerve.addSecondaryInputsTrueFielcentric(()->climber.generateSwerveInputs(swerve.getSwervePose())),
        swerve.turnToHeading(()->new Rotation2d(Degrees.of(-90))),
        intake.stop().asProxy()
      )
      .until(()->climber.rangefinders.isLinedUpL1())
      .withTimeout(6.0),
      climber.prepareForClimbL1().withTimeout(4.5)
      .alongWith(swerve.addSecondaryInputsTrueFielcentric(()->climber.generateSwerveInputs(swerve.getSwervePose()))),
      climber.climbL1()
    );
  }

  public Command climbAutoBlueLeft() {
    return Commands.sequence(
      pathing.followPathTeamFlipped(climbAutoRedSide).withTimeout(5.0).until(()->climber.rangefinders.isDetectable()),
      new ParallelCommandGroup(
        swerve.addSecondaryInputsTrueFielcentric(()->climber.generateSwerveInputs(swerve.getSwervePose())),
        swerve.turnToHeading(()->new Rotation2d(Degrees.of(90))),
        intake.stow().asProxy()
        )
      .until(()->climber.rangefinders.isLinedUpL1())
      .withTimeout(6.0),
      climber.prepareForClimbL1().withTimeout(4.5)
      .alongWith(swerve.addSecondaryInputsTrueFielcentric(()->climber.generateSwerveInputs(swerve.getSwervePose()))),
      climber.climbL1()
    );
  }

  public Command climbAutoRedRight() {
    climbAutoRedSide.mirror();
    return Commands.sequence(
      pathing.followPath(climbAutoRedSide),
      new ParallelCommandGroup(
        swerve.addSecondaryInputsTrueFielcentric(()->climber.generateSwerveInputs(swerve.getSwervePose())),
        swerve.turnToHeading(()->new Rotation2d(Degrees.of(90))),
        intake.stow().asProxy())
      // climber.prepareForClimbL1().withTimeout(1.0),
      // climber.climbL1()
    );
  }

  public Command PassingAutoBlueRIGHT() {
    shootIntitialPath.mirror();
    Path path = new Path("testingStraightUnder");
    path.mirror();
    return Commands.sequence(
      pathing.followPath(shootIntitialPath).withTimeout(3.0),
      pathing.followPath(path).withTimeout(20.0),
      pathing.followPath(path).withTimeout(20.0)
    );
  }

  public Command redDepot() {
    return Commands.sequence(
      basicShootInitial8().withTimeout(2.0),
      pathing.followPathTeamFlipped(new Path("depotAuto")).withTimeout(12),
      climbAutoRedLeft()
    );
  }

  public Command blueDepot() {
    return Commands.sequence(
      basicShootInitial8().withTimeout(2.0),
      pathing.followPath(new Path("depotAuto")).withTimeout(12),
      climbAutoBlueLeft()
    );
  }

  //////////////////////////////////////////
  /// SuperStructure State Commands ///////
  ////////////////////////////////////////

  // THESE PID COMMANDS ARE CHOPPED AND NOT TUNED THERE IS A REASON WE RUNNING
  // BLINE, DO NOT USE IN MOST CASES
  public Command driveToPose(double x, double y, double degrees, Command superState) {
    return driveToPose(new Pose2d(x, y, new Rotation2d(Degrees.of(degrees))), superState);
  }

  public Command driveToPoseSlowly(double x, double y, double degrees, Command superState, double maxVelocityMPS) {
    return driveToPoseSlowly(new Pose2d(x, y, new Rotation2d(Degrees.of(degrees))), superState, maxVelocityMPS);
  }

  public Command driveToPose(Pose2d targetPose, Command superState) {
    return swerve
      .pidToPoseInterpolated(
        () -> autoTeamFlippedPose(targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees()))
      .alongWith(superState)
    ;
  }

  public Command driveToPoseSlowly(Pose2d targetPose, Command superState, double maxVelocityMPS) {
    return swerve
      .pidToPose(
        () -> autoTeamFlippedPose(targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees()),
        maxVelocityMPS, Inches.of(5))
      .alongWith(superState)
      .until(() -> swerve.isOnTargetTranslate());
  }

  public Command stow() {
    return Commands.parallel(
      shooter.stow().asProxy(),
      spindexer.stop().asProxy(),
      intake.stop().asProxy()
    );
  }

  public Command pass() {
    return new ParallelCommandGroup(
      shooter.pass().asProxy(),
      spindexer.feedToShooterForce().asProxy(),
      intake.stop().asProxy()
    );
  }

  public Command shootAuto() {
    return new ParallelCommandGroup(
      shooter.shootHub().asProxy(),
      spindexer.feedToShooterForce().asProxy(),
      intake.stop().asProxy()
    );
  }

  public Command shootAutoNotBline() {
    return new ParallelCommandGroup(
      shooter.shootHub().asProxy(),
      spindexer.feedToShooterForce().asProxy(),
      intake.stop().asProxy()
    );
  }

  public Command intakeOnly() {
    return new ParallelCommandGroup(
      shooter.stow().asProxy(),
      spindexer.stop().asProxy(),
      intake.intake().asProxy()
    );
  }

  public Command intakeWhilePassing() {
    return new ParallelCommandGroup(
      shooter.pass().asProxy(),
      spindexer.feedToShooterForce().asProxy(),
      intake.intake().asProxy()
    );
  }

  public Command intakeWhileShooting() {
    return new ParallelCommandGroup(
      shooter.shootHubVelComp().asProxy(),
      spindexer.feedToShooterForce().asProxy(),
      intake.intake().asProxy()
    );
  }
}