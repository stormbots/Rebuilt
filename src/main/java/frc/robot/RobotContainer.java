// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.FieldBehaviour;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Photonvision.Photonvision;
import frc.robot.Subsystems.Questnav.QuestNav;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Spindexer.Spindexer;
import frc.robot.Subsystems.Swerve.Swerve;

public class RobotContainer {

  Swerve swerve = new Swerve();
  Photonvision photonvision = new Photonvision(swerve);
  QuestNav questnav = new QuestNav(swerve,photonvision);
  //TODO: TargetingSystem targetingsystem = new TargetingSystem(swerve);
  Shooter shooter = new Shooter(/* targetingsystem */);
  Intake Intake = new Intake();
  Spindexer spindexer = new Spindexer();
  FieldBehaviour fieldBehaviour = new FieldBehaviour();

  CommandXboxController driver = new CommandXboxController(0);
  CommandXboxController operator = new CommandXboxController(1);

  Field2d fieldBehaviourTestField = new Field2d();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    swerve.addDriverInputs(
      ()->driver.getLeftX(), 
      ()->driver.getLeftY(), 
      ()->driver.getRightX());

      SmartDashboard.putData("fieldBehaviourTestField",fieldBehaviourTestField);
      fieldBehaviourTestField.getObject("fakedrive").setPose(new Pose2d());

      //fieldBehaviourTestField.getObject("BlueTrench").setPoses(new Pose2d(),new Pose2d(1,1,new Rotation2d()));

     
      new Trigger(DriverStation::isEnabled)
      .whileTrue(
          swerve.addFieldInputs( ()->fieldBehaviour.getSwerveInputs(swerve.getSwervePose()) )
      );
  

  }


  public Command getAutonomousCommand() {
    //TODO: Get this from Autos.java instead
    return Commands.print("No autonomous command configured");
  }

}
