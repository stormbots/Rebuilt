// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import com.stormbots.CRTAbsoluteEncoder;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;

import java.io.SequenceInputStream;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
  // Photonvision photonvision = new Photonvision(swerve);
  // TargetingSystem targeting = new TargetingSystem(swerve);
  // Shooter shooter = new Shooter(targeting);
  // Intake intake = new Intake();
  Climber climber = new Climber();
  // Spindexer spindexer = new Spindexer(shooter.isReadyToAcceptFuel.and(swerve::isOnTargetAngle));
  // QuestNavSubsystem questnav = new QuestNavSubsystem(swerve);
  // Pathing pathing = new Pathing(swerve);
  // Signals signlas = new Signals();
  // // Bling bling = new Bling(); //TODO: Currently no bling lights on bot
  // FieldBehaviour fieldBehaviour = new FieldBehaviour();


  CommandXboxController driver = new CommandXboxController(0);
  CommandXboxController operator = new CommandXboxController(1);
  Path testingPath = new Path("goCollect");
  Double rpm = 2600.0;
  Double hoodAngle = 25.0;

  public RobotContainer() {
    SmartDashboard.putNumber("robotContainer/flywheelrpm", rpm);
    SmartDashboard.putNumber("robotContainer/hoodAngle", hoodAngle);

    // HopperSensors.getInstance(); //Ensure this always exists and is updating
    // questnav.setQuestPose(new Pose3d(swerve.swerveDrive.getPose().getX(), swerve.swerveDrive.getPose().getY(), 0.0, new Rotation3d(0.0, 0.0, 0.0)));
    
    configureDriverBindings();
    configureOperatorBindings();

    
    //CRTAbsoluteEncoder.getInstance().sync();

    // new Trigger(DriverStation::isEnabled){
      //wait for PV to have seen a tag
      //set questnav position
    // }

    //QuestNav initialization?
    //THIS IS VERY JANK, FIX LATER, should be part of the auto starting sequence, should setQuestPose THEN wantToTrack, this was dumb
    // driver.povDown().onTrue(new InstantCommand(()->questnav.setQuestPose(new Pose3d(0.0, 7.5, 0.0, new Rotation3d(0.0, 0.0, 0.0)))));
    // driver.povUp().onTrue(new InstantCommand(()->questnav.wantToTrack()));

    // new Trigger(DriverStation::isEnabled)
    // .whileTrue(
    //   swerve.addFieldInput( ()->fieldBehaviour.getSwerveInputs(swerve.getSwervePose()) )
    // );

  }

  private void configureDriverBindings() {
    swerve.setDefaultCommand(swerve.setPrimaryInputs(
      ()->-driver.getLeftY(), 
      ()->-driver.getLeftX(), 
      ()->-driver.getRightX()
    ));


    //TODO: PROGRAMMING DEBUG BUTTONS CODE REMOVE ME

    driver.start().onTrue(
      Commands.sequence(
        swerve.testZeroPose().withTimeout(0.1)
        // new InstantCommand(()->questnav.setQuestPose(new Pose3d(swerve.getSwervePose().getX(), swerve.getSwervePose().getY(), 0.0, new Rotation3d(0.0, 0.0, 0.0)))).withTimeout(0.1),
        // new InstantCommand(()->questnav.wantToTrack())
        )
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

    // driver.x() // extend intake // This button is useless and will never be used
    // .whileTrue(intake.intake()).whileFalse(intake.stop());

    // driver.y().whileTrue(intake.eject()); //intake eject //also will never be used

    // // driver.(back left paddle) //global stow/defense mode

    // driver.povUp().whileTrue(shooter.testHome());
    // //  driver.y().whileTrue(shooter.testSetHoodAngle(Degrees.of(30)));

    driver.rightBumper().whileTrue(Commands.parallel(
      climber.prepareForClimbL1(),
      // swerve.turnToHeading(()->new Rotation2d(Degree.of(90))),
      swerve.addSecondaryInputsTrueFielcentric(()->climber.generateInputs(swerve.getSwervePose())),
      Commands.none()
    ).until(climber::isLinedUpWithL1)
    .andThen(climber.climbL1())
    );

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
    // operator.povUp().whileTrue(spindexer.unclog()); // shake dye rotor / unclog

    // operator.povDown().whileTrue(intake.eject()); // intake.eject()

    // operator.x() // shoot + hopper feed
    // .whileTrue(shooter.shootHub())
    // .whileTrue(spindexer.feedToShooter())
    // .whileTrue(swerve.turnToHeading(()->{
    //   return targeting.getHeadingToTarget(swerve.getSwervePose().getTranslation(), targeting.getTarLockTemp().getTranslation()).plus(Rotation2d.k180deg);
    // }));


    // operator.a().whileTrue(climber.stow()); // global stow (unnecessary, this is default)

    // operator.b()// passing: Face driver station wall and launch at fixed rpm/angle/distance
    // .whileTrue(shooter.pass())
    // .whileTrue(spindexer.feedToShooter())
    //.whileTrue(swerve.turnToHeading(()->new Rotation2d()));

    // operator.back() // re-home intake, hood, turret? hood? not doing this rn

    // Align to the climb process/button
    // align up/down swerve.setAngle()
    // addRangeFinderOutputs
    //once in place
    //climb

    


  }


  
  

  public Command getAutonomousCommand() {
    //TODO: Get this from Autos.java instead
    Commands.sequence(
      //home everything
      //do the rest of the sequence
    );
    return Commands.print("No autonomous command configured");
  }
}
