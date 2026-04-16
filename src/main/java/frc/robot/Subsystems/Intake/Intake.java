// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake;

import static edu.wpi.first.units.Units.Degree;

import com.stormbots.CRTAbsoluteEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.Intake.IntakeExtension.IntakeExtension;
import frc.robot.Subsystems.Intake.Rollers.Rollers;

/** Add your docs here. */
public class Intake extends SubsystemBase {
  private IntakeExtension left = new IntakeExtension(9,false);
  private IntakeExtension right = new IntakeExtension(10,true);
  private Rollers rollers = new Rollers();
  private IntakeVisualizer visual = new IntakeVisualizer();

  public Trigger isDeployed = new Trigger(()->
    left.getAngle().in(Degree) < 45
    && right.getAngle().in(Degree) < 45
  );

  public Intake(){
    CRTAbsoluteEncoder.getInstance().setEncoder2(left.getAbsoluteEncoder());
  }

  @Override
  public void periodic(){
    visual.update(left.getAngle(), rollers.getPosition(),rollers.getVelocity());
    SmartDashboard.putNumber("shooter/turret/e2", left.getAbsoluteEncoder().getPosition());
  }

  public Command intake(){
    return Commands.parallel(
      new WaitCommand(0.05).andThen(rollers.intake()),
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

  public Command bringUp(){
    var upRollers = Commands.parallel(
      left.stow(),
      right.stow(),
      rollers.intake()
    )
    .withTimeout(0.5)
    .until(()->{
      return left.getAngle().in(Degree) > 45
      && right.getAngle().in(Degree) > 45;
    })
    ;

    return Commands.sequence(
      upRollers,
      up()
    )
    .withName("bringUp")
    ;
  }  
  
  public Command up(){
    return Commands.parallel(
      rollers.stop(),
      left.up(),
      right.up()
    )
    .withName("Stop")
    ;
  }

  public Command stow(){
    return Commands.parallel(
      rollers.stop(),
      left.stow(),
      right.stow()
    )
    .withName("Stow")
    ;
  }

  public Command shootingStow(){
    return Commands.parallel(
      rollers.stop(),
      left.upTest(left.getCurrentCommand().getName()),
      right.upTest(right.getCurrentCommand().getName())
    )
    .withName("ShootingUp")
    ;
  }

  public Command testRollers(){
    return Commands.parallel(
      rollers.setVoltage(2)
    );
  }
}
