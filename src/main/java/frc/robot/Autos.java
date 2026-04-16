// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
  SendableChooser<Boolean> sideChooser = new SendableChooser<>();

  // Commonly used paths, My thinking is we shouldn't do this like this, 
  // caused flipping issues for the defense prep auto because the path gets mirrored and then swapped back
  Path centerShootPath = new Path("CenterShootAutoV2");
  Path shootIntitialPath = new Path("shootInitial");
  Path postShootToClimb = new Path("postShootingToClimb");
  Path climbAutoRedSide = new Path("climbAuto");
  Path climbAutoV2 = new Path("ClimbAutoV2");

  Boolean side = true;//True is Right Side

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
    // autoChooser.addOption("Red Depot", this::redDepotNoSOTM);
    // autoChooser.addOption("Blue Depot", this::blueDepotNoSOTM);
    // autoChooser.addOption("Red Depot V2", this::redDepotButBetter);
    // autoChooser.addOption("Blue Depot V2", this::blueDepotButBetter);

    // autoChooser.addOption("ShootOnlyEight", this::basicShoot);

    // autoChooser.addOption("RL Center Auto", this::CenterShootAutoRedLEFT);
    // autoChooser.addOption("BL Center Auto", this::CenterShootAutoBlueLEFT);
    // autoChooser.addOption("RR  Center Auto", this::CenterShootAutoRedRIGHT);
    // autoChooser.addOption("BR Center Auto", this::CenterShootAutoBlueRIGHT);

    autoChooser.addOption("Center Auto", this::CenterShootAuto);
    autoChooser.addOption("Defense Prep", this::thisIsStupid);
    autoChooser.addOption("Depot NO SOTM", this::depotNoSOTM);
    autoChooser.addOption("Depot V2", this::depotButBetter);
    

    autoChooser.setDefaultOption("Select Auto", () -> new InstantCommand());
    // autoChooser.addOption("VV UNTESTED VV", () -> new InstantCommand());
    // autoChooser.addOption("ClimbAutoChoppedWhyAreWeDoingThisIWannaShootSoBad", this::climbAutoBlueLeft); // best name ever

    // autoChooser.onChange((cs)->System.out.print("Running selected auto " +cs.get().getName()));


    SmartDashboard.putData("AutoSelector/chooser", autoChooser);

    sideChooser.addOption("right", true);
    sideChooser.addOption("left", false);
    sideChooser.onChange((isRight) -> Pathing.isRightAuto = isRight);
    
    sideChooser.setDefaultOption("Select A Side", true);
    SmartDashboard.putData("AutoSelector/sideChooser", sideChooser);
    // SmartDashboard.putBoolean("AutoSelector/L-R Side", side);

  }

  // Get Auto Commandx
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

  //YAYYYYYAYYYAYY THESE WORK AT START OF BLINE AUTOS NOW!!!!!!!!, Still not really running anything other than the shoot

  public Command basicShoot() {
    return new ParallelCommandGroup(
      swerve.turnToHeadingWithinTurretRange(()->targeting.getHeadingToTarget(swerve.getSwervePose().getTranslation(), targeting.getHubTarget())
          .plus(Rotation2d.k180deg)),
      shootAuto()
    );
  }

  // ALL OF THESE AUTOS ARE THE GOOD BLINE TYPE

  public Command CenterShootAuto(){
    
    return Commands.sequence(
      basicShoot().withTimeout(2.25),
      pathing.followPath(centerShootPath).withTimeout(12),
      pathing.followPath(postShootToClimb).withTimeout(2.0),
      climbAuto()
    );
  }


  public Command thisIsStupid(){
    return Commands.sequence(
      basicShoot().withTimeout(2.25),
      pathing.followPath(new Path("defensePrep"))
    );
  }

  public Command climbAuto(){
    //TODO:Make heading lock flip 90 or -90 off driver station
    return Commands.sequence(
      pathing.followPath(climbAutoV2).withTimeout(3.0).until(climber.rangefinders.isRightChecked),
      new ParallelCommandGroup(
        swerve.addSecondaryInputsTrueFielcentric(()->climber.generateSwerveInputs(swerve.getSwervePose())),
        // swerve.turnToHeading(()->new Rotation2d(Degrees.of(-90))),
        intake.stop().asProxy()
      )
      .withTimeout(3.5),
      climber.prepareForClimbL1().withTimeout(3.5),
      climber.climbL1()
    );
  }

  public Command depotButBetter(){
    return Commands.sequence(
      basicShoot().withTimeout(2.25),
      pathing.followPath(new Path("depotAutoV2")).withTimeout(13),
      climbAuto()
    );
  }

  public Command depotNoSOTM(){
    return Commands.sequence(
      basicShoot().withTimeout(2.25),
      pathing.followPath(new Path("depotAutoSweep1")),
      basicShoot().withTimeout(2.5),
      pathing.followPath(new Path("depotAutoSweep2")),
      basicShoot().alongWith(swerve.stop()).withTimeout(2.5),
      climbAuto()
    )
    // .withName("Red Depot Auto NOSOTM")
    ;
  }


  

  //////////////////////////////////////////
  /// SuperStructure State Commands ///////
  ////////////////////////////////////////
  //WE AINT NORMALLY USING TS//
 
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
      spindexer.feedToShooter().asProxy(),
      intake.stop().asProxy()
    );
  }

  public Command shootAuto() {
    return new ParallelCommandGroup(
      shooter.shootHub().asProxy(),
      spindexer.feedToShooter().asProxy(),
      intake.stop().asProxy()
    );
  }

  public Command shootAutoNotBline() {
    return new ParallelCommandGroup(
      shooter.shootHub().asProxy(),
      spindexer.feedToShooter().asProxy(),
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