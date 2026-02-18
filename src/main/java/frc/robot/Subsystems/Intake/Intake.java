// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Intake.IntakeExtension.IntakeExtension;
import frc.robot.Subsystems.Intake.Rollers.Rollers;

/** Add your docs here. */
public class Intake extends SubsystemBase {
  private IntakeExtension intakeExtension = new IntakeExtension();
  private Rollers rollers = new Rollers();
  private IntakeVisualizer visual = new IntakeVisualizer();

  public Intake(){
  }

  @Override
  public void periodic(){
    visual.update(intakeExtension.getAngle(), rollers.getPosition(),rollers.getVelocity());
  }

  public Command intake(){
    return Commands.parallel(
      rollers.intake(),
      intakeExtension.down()
    );
  };

  public Command eject(){
    return Commands.parallel(
      rollers.eject(),
      intakeExtension.down()
    );
  }

  public Command stop(){
    return Commands.parallel(
      rollers.stop(),
      intakeExtension.up()
    );
  }

  public Command testRollers(){
    return Commands.parallel(
      rollers.setVoltage(2)
    );
  }

  public Command testDown(){
    return Commands.parallel(
      intakeExtension.down()
    );
  }
  
}
