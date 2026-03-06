// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import com.stormbots.CRTAbsoluteEncoder;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.FieldBehaviour;
import frc.robot.Subsystems.Climber.Climber;
import frc.robot.Subsystems.HopperSensors.HopperSensors;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Lighting.Signals;
import frc.robot.Subsystems.Photonvision.Photonvision;
import frc.robot.Subsystems.Questnav.QuestNavSubsystem;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Spindexer.Spindexer;
import frc.robot.Subsystems.Swerve.Pathing;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.TargetingSystem.TargetingSystem;
import frc.robot.lib.BLine.Path;

public class RobotContainer {

  Swerve swerve = new Swerve();
  Photonvision photonvision = new Photonvision(swerve);
  TargetingSystem targeting = new TargetingSystem(swerve);
  Shooter shooter = new Shooter(targeting);
  Intake intake = new Intake();
  Climber climber = new Climber();
  Spindexer spindexer = new Spindexer(shooter.isReadyToAcceptFuel.and(swerve::isOnTargetAngle));
  QuestNavSubsystem questnav = new QuestNavSubsystem(swerve);
  Pathing pathing = new Pathing(swerve);
  Signals signlas = new Signals();
  Autos autos = new Autos(swerve, shooter, intake, questnav, spindexer, pathing, targeting);
  // Bling bling = new Bling(); //TODO: Currently no bling lights on bot
  FieldBehaviour fieldBehaviour = new FieldBehaviour();


  CommandXboxController driver = new CommandXboxController(0);
  CommandXboxController operator = new CommandXboxController(1);
  CommandXboxController debug = new CommandXboxController(3);
  Path testingPath = new Path("goCollect");
  Double rpm = 2600.0;
  Double hoodAngle = 25.0;

  public void syncQuestPose (){
    questnav.setQuestPose(new Pose3d(swerve.getSwervePose()));
  };

  public RobotContainer() {
    SmartDashboard.putNumber("robotContainer/flywheelrpm", rpm);
    SmartDashboard.putNumber("robotContainer/hoodAngle", hoodAngle);

    HopperSensors.getInstance(); //Ensure this always exists and is updating
    questnav.setQuestPose(new Pose3d(swerve.swerveDrive.getPose().getX(), swerve.swerveDrive.getPose().getY(), 0.0, new Rotation3d(0.0, 0.0, 0.0)));
    
    configureDriverBindings();
    configureOperatorBindings();
    configureDebugBindings();

    
    CRTAbsoluteEncoder.getInstance().sync();

    //QuestNav initialization?
    //THIS IS VERY JANK, FIX LATER, should be part of the auto starting sequence, should setQuestPose THEN wantToTrack, this was dumb
    driver.povDown().onTrue(new InstantCommand(()->questnav.setQuestPose(new Pose3d(0.0, 7.5, 0.0, new Rotation3d(0.0, 0.0, 0.0)))));
    driver.povUp().onTrue(new InstantCommand(()->questnav.wantToTrack(true)));

    // new Trigger(DriverStation::isEnabled)
    // .whileTrue(
    //   swerve.addFieldInput( ()->fieldBehaviour.getSwerveInputs(swerve.getSwervePose()) )
    // );

    // syncQuestPose.runsWhenDisabled();

    // new Trigger(()->Timer.getFPGATimestamp() > 1)
    // .and(DriverStation::isDisabled)
    // .whileTrue(syncQuestPose);

    //while disabled
    //and see target
    //update quest pose
    //onEnable

  }

  private void configureDebugBindings(){
    debug.a().whileTrue(pathing.followPath(new Path("example_a")));

  }

  private void configureDriverBindings() {
    swerve.setDefaultCommand(swerve.setPrimaryInputs(
      ()->-driver.getLeftY()*2.0, 
      ()->-driver.getLeftX()*2.0, 
      ()->-driver.getRightX()*2.0
    ));


    //TODO: PROGRAMMING DEBUG BUTTONS CODE REMOVE ME

    driver.start().onTrue(
      Commands.sequence(
        swerve.testZeroPose().withTimeout(0.1),
        new InstantCommand(()->questnav.setQuestPose(new Pose3d(swerve.getSwervePose().getX(), swerve.getSwervePose().getY(), 0.0, new Rotation3d(0.0, 0.0, 0.0)))).withTimeout(0.1),
        new InstantCommand(()->questnav.wantToTrack(true)))
      );

    // driver.rightTrigger().whileTrue(shooter.shootHub());
    // driver.leftTrigger().whileTrue(spindexer.feedToShooter());
    

    // driver.povLeft().whileTrue(shooter.testHome());
    // driver.a().whileTrue(swerve.turnToHeading(()->{
    //   return targeting.getHeadingToTarget(swerve.getSwervePose().getTranslation(), targeting.getTarLockTemp().getTranslation()).plus(Rotation2d.k180deg);
    // }));


    // driver.povDown().whileTrue(shooter.testHome());
    // driver.a().whileTrue(swerve.turnToHeading(()->{
    //   return targeting.getHeadingToTarget(swerve.getSwervePose().getTranslation(), targeting.getTarLockTemp().getTranslation()).plus(Rotation2d.k180deg);
    // }));
    // END PROGRAMMING DEBUG BUTTONS


    driver.leftBumper().whileTrue(
      swerve.setPrimaryInputs(
      ()->-driver.getLeftY()/3.0, 
      ()->-driver.getLeftX()/3.0, 
      ()->-driver.getRightX()/3.0
    )
    );

    // driver.start().onTrue(swerve.zeroGyro()); //zero heading, occulus implementation added later

    driver.a().whileTrue(swerve.turnToHeading(()->new Rotation2d())); // face swerve away from driver
    // driver.b() //TODO: face any 45 to prepare for bump crossing

    driver.x() // extend intake // This button is useless and will never be used
    .whileTrue(intake.intake())
    // .whileTrue(spindexer.spinDyeRotor())
    .whileFalse(intake.stop());

    driver.y().whileTrue(intake.eject()); //intake eject //also will never be used

    // driver.(back left paddle) //global stow/defense mode

    driver.povUp().whileTrue(shooter.testHome());
    //  driver.y().whileTrue(shooter.testSetHoodAngle(Degrees.of(30)));



  }

  private void configureOperatorBindings() {


    // operator.leftTrigger() // Stage2 hook down, lock out if not end of match
    // operator.leftBumper().whileTrue(climber.setStage1Voltage(12));
    // operator.leftTrigger().whileTrue(climber.setStage1Voltage(-12));

    operator.leftBumper()
      .whileTrue(climber.prepareForClimbL1())
      .onFalse(climber.climbL1())
    ;

    // operator.rightBumper() stage2 hook up, lock out if not end of match  
    // operator.rightBumper().whileTrue(climber.setStage2Voltage(12));
    // // operator.rightTrigger() stage2 hook down, lock out if not end of match
    // operator.rightTrigger().whileTrue(climber.setStage2Voltage(-12));
    operator.povUp().whileTrue(spindexer.unclog()); // shake dye rotor / unclog

    // operator.povRight()
    // .whileTrue(intake.intake())
    // .whileTrue(spindexer.spinDyeRotor())
    // .whileFalse(intake.stop());

    operator.povDown().whileTrue(intake.eject()); // intake.eject()

    operator.x() // shoot + hopper feed
    .whileTrue(shooter.shootHub())
    .whileTrue(spindexer.feedToShooter())
    .whileTrue(swerve.turnToHeadingNiche(()->{
      return targeting.getHeadingToTarget(swerve.getSwervePose().getTranslation(), targeting.getHubTarget()).plus(Rotation2d.k180deg);
    }))
    ;

    double bool =  swerve.getSwervePose().getRotation().getMeasure()
      .isNear(Degrees.of(90), Degrees.of(90)) ? 90 : -90;

    operator.a().whileTrue(climber.stow()); // global stow (unnecessary, this is default)
    operator.povLeft().whileTrue(climber.goHome());

    operator.b()// passing: Face driver station wall and launch at fixed rpm/angle/distance
    .whileTrue(shooter.pass())
    .whileTrue(spindexer.feedToShooter())
    .whileTrue(swerve.turnToHeadingNiche(()->{
      return targeting.getHeadingToTarget(swerve.getSwervePose().getTranslation(), targeting.getPassTarget()).plus(Rotation2d.k180deg);
    }))
    ;
    // Commands.either(/*-90 */, /* 90 */, /*whichever is closer */)
    // Commands.either(
    //   swerve.turnToHeading(()->new Rotation2d()), 
    //   swerve.turnToHeading(()->new Rotation2d()), 
    //   swerve.getSwervePose().getRotation())

    // operator.back() // re-home intake, hood, turret? hood? not doing this rn

  }


  public Command getAutonomousCommand() {
    //TODO: Get this from Autos.java instead
    
    return Commands.print("No autonomous command configured");
  }
}
