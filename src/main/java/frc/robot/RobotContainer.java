// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.stormbots.CRTAbsoluteEncoder;

import static edu.wpi.first.units.Units.Degrees;


import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.FieldBehaviour;
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
  Spindexer spindexer = new Spindexer(shooter.isReadyToAcceptFuel);
  QuestNavSubsystem questnav = new QuestNavSubsystem(swerve);
  Pathing pathing = new Pathing(swerve);
  Signals signlas = new Signals();
  // Bling bling = new Bling(); //TODO: Currently no bling lights on bot
  FieldBehaviour fieldBehaviour = new FieldBehaviour();


  CommandXboxController driver = new CommandXboxController(0);
  CommandXboxController operator = new CommandXboxController(1);
  Path testingPath = new Path("goCollect");
  Double rpm = 2600.0;
  Double hoodAngle = 25.0;

  public RobotContainer() {
    SmartDashboard.putNumber("robotContainer/flywheelrpm", rpm);
    SmartDashboard.putNumber("robotContainer/hoodAngle", hoodAngle);

    HopperSensors.getInstance(); //Ensure this always exists and is updating
    questnav.setQuestPose(new Pose3d(swerve.swerveDrive.getPose().getX(), swerve.swerveDrive.getPose().getY(), 0.0, new Rotation3d(0.0, 0.0, 0.0)));
    configureBindings();

    CRTAbsoluteEncoder.getInstance().sync();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(swerve.setPrimaryInputs(
      ()->-driver.getLeftY(), 
      ()->-driver.getLeftX(), 
      ()->-driver.getRightX()
    ));

    // driver.a().whileTrue(swerve.turnToHeading(new Rotation2d(0.0, 0.0)));
    // driver.b().whileTrue(swerve.turnToHeading(new Rotation2d()));
    // if(Robot.isSimulation()){
    //   //Make it "drive right" on the sim field using default Red1
    //   swerve.setDefaultCommand(swerve.addDriverInputs(
    //     ()->-driver.getLeftX()/2.0, 
    //     ()->driver.getLeftY()/2.0, 
    //     ()->-driver.getRightX()/2.0
    //   ));
    // }

    // driver.povRight().whileTrue(shooter.shoot(()->new TargetingSystem.ShooterState(
    //   Degrees.of(0), 
    //   Degrees.of(SmartDashboard.getNumber("robotContainer/hoodAngle", hoodAngle)), 
    //   SmartDashboard.getNumber("robotContainer/flywheelrpm", rpm)))
    // );

    driver.rightTrigger().whileTrue(shooter.shootHub());
    driver.leftTrigger().whileTrue(spindexer.feedToShooter());
    

    driver.povLeft().whileTrue(shooter.testHome());
    driver.a().whileTrue(swerve.turnToHeading(()->{
      return targeting.getHeadingToTarget(swerve.getSwervePose().getTranslation(), targeting.getTarLockTemp().getTranslation()).plus(Rotation2d.k180deg);
    }));
    // driver.b().whileTrue(swerve.turnToHeading(new Rotation2d(0.0)));

    // driver.povUp().whileTrue(shooter.testSetHoodAngle(Degrees.of(30)));
    // driver.leftBumper().whileTrue(shooter.testSetTurretAngle(Degrees.of(30)));
    // driver.rightBumper().whileTrue(shooter.testSetTurretAngle(Degrees.of(0)));


    //  driver.povUp().whileTrue(shooter.testHome());
    //  driver.y().whileTrue(shooter.testSetHoodAngle(Degrees.of(30)));
    // swerve.setDefaultCommand(swerve.addDriverInputs(
    //   ()->-driver.getLeftY(), 
    //   ()->-driver.getLeftX(), 
    //   ()->-driver.getRightX()
    // ));

    //TODO Add intakes to controller
    driver.x().whileTrue(intake.intake());
    driver.y().whileTrue(intake.stop());


    // new Trigger(DriverStation::isEnabled)
    // .whileTrue(
    //   swerve.addFieldInput( ()->fieldBehaviour.getSwerveInputs(swerve.getSwervePose()) )
    // );

    // driver.b().whileTrue(spindexer.feedToShooter());

    driver.start().onTrue(swerve.testZeroPose());


    //THIS IS VERY JANK, FIX LATER, should be part of the auto starting sequence, should setQuestPose THEN wantToTrack, this was dumb
    driver.povDown().onTrue(new InstantCommand(()->questnav.setQuestPose(new Pose3d(0.0, 7.5, 0.0, new Rotation3d(0.0, 0.0, 0.0)))));
    driver.povUp().onTrue(new InstantCommand(()->questnav.wantToTrack()));

    // driver.x().whileTrue(pathing.followPath(testingPath))
    // driver.y().onTrue(swerve.addAutoInputs(()->0.0, ()->0.0, ()->0.0));
    if(Robot.isSimulation()){
      //Make it "drive right" on the sim field using default Red1
      swerve.setDefaultCommand(swerve.setPrimaryInputs(
        ()->-driver.getLeftX(), 
        ()->driver.getLeftY(), 
        ()->-driver.getRightX()
      ));
    }

  }


  public Command getAutonomousCommand() {
    //TODO: Get this from Autos.java instead
    return Commands.print("No autonomous command configured");
  }
}
