// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import com.stormbots.CRTAbsoluteEncoder;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.HopperSensors.HopperSensors;
import frc.robot.Subsystems.Intake.Intake;
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

  CommandXboxController driver = new CommandXboxController(0);
  CommandXboxController operator = new CommandXboxController(1);
  Path testingPath = new Path("goCollect");
  public RobotContainer() {
    HopperSensors.getInstance(); //Ensure this always exists and is updating
    questnav.setQuestPose(new Pose3d(swerve.swerveDrive.getPose().getX(), swerve.swerveDrive.getPose().getY(), 0.0, new Rotation3d(0.0, 0.0, 0.0)));
    configureBindings();

    CRTAbsoluteEncoder.getInstance().sync();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(swerve.addDriverInputs(
      ()->-driver.getLeftY(), 
      ()->-driver.getLeftX(), 
      ()->-driver.getRightX()
    ));
    // if(Robot.isSimulation()){
    //   //Make it "drive right" on the sim field using default Red1
    //   swerve.setDefaultCommand(swerve.addDriverInputs(
    //     ()->-driver.getLeftX()/2.0, 
    //     ()->driver.getLeftY()/2.0, 
    //     ()->-driver.getRightX()/2.0
    //   ));
    // }

    // driver.a().whileTrue(shooter.testSetFlywheelRPM(1000));
    driver.a().whileTrue(shooter.testFlywheelVoltage(4));
    driver.b().whileTrue(spindexer.feedToShooter());
    driver.x().whileTrue(spindexer.setVoltages(8.0, 0.0));
    driver.y().whileTrue(spindexer.setVoltages(0.0, 8.0));

    driver.povDown().whileTrue(shooter.testHome());
    driver.povUp().whileTrue(shooter.testSetHoodAngle(Degrees.of(30)));


    //  driver.povUp().whileTrue(shooter.testHome());
    //  driver.y().whileTrue(shooter.testSetHoodAngle(Degrees.of(30)));
    // swerve.setDefaultCommand(swerve.addDriverInputs(
    //   ()->-driver.getLeftY(), 
    //   ()->-driver.getLeftX(), 
    //   ()->-driver.getRightX()
    // ));

    //THIS IS VERY JANK, FIX LATER, should be part of the auto starting sequence, should setQuestPose THEN wantToTrack, this was dumb
    // driver.a().onTrue(swerve.zeroGyro());
    // driver.a().onTrue(new InstantCommand(()->questnav.setQuestPose(new Pose3d(swerve.swerveDrive.getPose().getX(), swerve.swerveDrive.getPose().getY(), 0.0, new Rotation3d(0.0, 0.0, 0.0)))));
    // driver.b().onTrue(new InstantCommand(()->questnav.wantToTrack()));
    // driver.x().whileTrue(pathing.followPath(testingPath));
    // driver.y().onTrue(swerve.addAutoInputs(()->0.0, ()->0.0, ()->0.0));
    if(Robot.isSimulation()){
      //Make it "drive right" on the sim field using default Red1
      swerve.setDefaultCommand(swerve.addDriverInputs(
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
