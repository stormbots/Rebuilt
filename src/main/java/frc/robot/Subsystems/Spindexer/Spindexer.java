// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Spindexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Spindexer.DyeRotor.DyeRotor;
import frc.robot.Subsystems.Spindexer.UpGoer.UpGoer;

public class Spindexer extends SubsystemBase{
  private DyeRotor dyeRotor = new DyeRotor();
  private UpGoer upGoer = new UpGoer();
  SpindexerVisual mech = new SpindexerVisual(); 

  /** Creates a new Spindexer. */
  public Spindexer(){
  }

  private Command spinDyeRotor(){
    return dyeRotor.spin();
  }

  public Command spinUpGoer(){
    return upGoer.spin();
  }

  public Command intake(){
    return dyeRotor.spin();
  }

  public Command feedToShooter(){
    return Commands.parallel(
      dyeRotor.load(),
      upGoer.load()
    );
  }

  public Command unclog(){
    return Commands.parallel(
      dyeRotor.spinBackwards(),
      upGoer.unclog()
    );
  }

  public Command stop(){
    return Commands.parallel(
      dyeRotor.stop(),
      upGoer.stop()
    );
  }

  private void setVoltages(){
    dyeRotor.setVoltage(5);
    upGoer.setVoltage(5);
  }

  public void periodic(){
    mech.update(dyeRotor.getPosition(), upGoer.getVelocity());
  }
}
