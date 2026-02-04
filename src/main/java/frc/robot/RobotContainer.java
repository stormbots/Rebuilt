// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Lighting.Lighting;
import frc.robot.Subsystems.Photonvision.Photonvision;
import frc.robot.Subsystems.Questnav.QuestNavSubsystem;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Spindexer.Spindexer;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Pathing;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

public class RobotContainer {

  Swerve swerve = new Swerve();
  Photonvision photonvision = new Photonvision(swerve);
  QuestNavSubsystem questnav = new QuestNavSubsystem(swerve);
  //TODO: TargetingSystem targetingsystem = new TargetingSystem(swerve);
  Shooter shooter = new Shooter(/* targetingsystem */);
  Intake Intake = new Intake();
  Spindexer spindexer = new Spindexer();
  Pathing pathing = new Pathing(swerve);

  CommandXboxController driver = new CommandXboxController(0);
  CommandXboxController operator = new CommandXboxController(1);
  Path testingPath = new Path("triangle");
  public RobotContainer() {
    configureBindings();
  }



  private void configureBindings() {
    swerve.setDefaultCommand(swerve.addDriverInputs(
      ()->-driver.getLeftY(), 
      ()->-driver.getLeftX(), 
      ()->-driver.getRightX()
    ));
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
