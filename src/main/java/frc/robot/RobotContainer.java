// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Climber.Climber;
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
  Climber climber = new Climber();

  CommandXboxController driver = new CommandXboxController(0);
  CommandXboxController operator = new CommandXboxController(1);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {


    //Test code but it works
    driver.povLeft().whileTrue(climber.setStage1Voltage(-12)); //stage1 down
    driver.povUp().whileTrue(climber.setStage1Voltage(12)); //stage1 up
    driver.povDown().whileTrue(climber.setStage2Voltage(-12)); //stage2 down
    driver.povRight().whileTrue(climber.setStage2Voltage(12)); //stage2 up

    driver.rightBumper()
    .whileTrue(climber.prepareForClimbL1())
    .onFalse(climber.climbL1().withTimeout(10))
    ;

    driver.start().whileTrue(climber.goHome());
  }


  public Command getAutonomousCommand() {
    //TODO: Get this from Autos.java instead
    return Commands.print("No autonomous command configured");
    // return climber.goHome();
  }

}
