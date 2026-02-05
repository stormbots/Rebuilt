// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.stormbots.CRTAbsoluteEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.HopperSensors.HopperSensors;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Photonvision.Photonvision;
import frc.robot.Subsystems.Questnav.QuestNav;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Spindexer.Spindexer;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.TargetingSystem.TargetingSystem;

public class RobotContainer {

  Swerve swerve = new Swerve();
  Photonvision photonvision = new Photonvision(swerve);
  QuestNav questnav = new QuestNav(swerve,photonvision);
  TargetingSystem targeting = new TargetingSystem(swerve);
  Shooter shooter = new Shooter(targeting);
  Intake intake = new Intake();
  Spindexer spindexer = new Spindexer();

  CommandXboxController driver = new CommandXboxController(0);
  CommandXboxController operator = new CommandXboxController(1);

  public RobotContainer() {
    HopperSensors.getInstance(); //Ensure this always exists and is updating
    configureBindings();

    CRTAbsoluteEncoder.getInstance().sync();
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
        ()->-driver.getLeftX()/2.0, 
        ()->driver.getLeftY()/2.0, 
        ()->-driver.getRightX()/2.0
      ));
    }

    driver.a().whileTrue(intake.smartIntake());
    driver.b().whileTrue(shooter.simGetLaunchCommand());
  }

  public Command getAutonomousCommand() {
    //TODO: Get this from Autos.java instead
    return Commands.print("No autonomous command configured");
  }

}
