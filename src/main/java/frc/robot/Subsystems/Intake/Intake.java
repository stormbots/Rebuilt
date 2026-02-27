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
  private IntakeExtension left = new IntakeExtension(9,false);
  private IntakeExtension right = new IntakeExtension(10,true);
  private Rollers rollers = new Rollers();
  private IntakeVisualizer visual = new IntakeVisualizer();

  public Intake(){
  }

  @Override
  public void periodic(){
    visual.update(left.getAngle(), rollers.getPosition(),rollers.getVelocity());
  }

  public Command intake(){
    return Commands.parallel(
      rollers.intake(),
      left.down(),
      right.down()
    )    
    .withName("Intake")
    ;    
  }

  public Command eject(){
    return Commands.parallel(
      rollers.eject(),
      left.down(),
      right.down()
    );
  }

  public Command stop(){
    return Commands.parallel(
      rollers.stop(),
      left.up(),
      right.up()
    )
    .withName("Stop")
    ;
  }

  public Command testRollers(){
    return Commands.parallel(
      rollers.setVoltage(2)
    );
  }
  
}
