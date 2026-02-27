// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Spindexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.Spindexer.DyeRotor.DyeRotor;
import frc.robot.Subsystems.Spindexer.UpGoer.UpGoer;

public class Spindexer extends SubsystemBase{
  private DyeRotor dyeRotor = new DyeRotor();
  private UpGoer upGoer = new UpGoer();
  SpindexerVisual mech = new SpindexerVisual();
  private Trigger shooterReady; 
  
  /** Creates a new Spindexer. */
  public Spindexer(Trigger shooterReady){
      this.shooterReady = shooterReady;
  }

  public Command spinDyeRotor(){
    return dyeRotor.feed();
  }

  public Command spinUpGoer(){
    return upGoer.feed();
  }

  public Command intake(){
    //TODO impliment spin, but lower power, current, or agitate
    return dyeRotor.intake();
  }

  public Command feedToShooter(){
    var feed = Commands.parallel(
      dyeRotor.feed(),
      upGoer.feed()
    );

    return Commands.repeatingSequence(
      stop().until(shooterReady),
      feed.until(shooterReady.negate())
    );
  }

  public Command unclog(){
    var reverse = Commands.parallel(
      dyeRotor.unclog(),
      upGoer.stop()
    ).withTimeout(0.5);

    var forward = Commands.parallel(
      dyeRotor.feed(),
      upGoer.stop()
    );

    return Commands.repeatingSequence(
      reverse,
      forward
    );
  }

  public Command stop(){
    return Commands.parallel(
      dyeRotor.stop(),
      upGoer.stop()
    );
  }

  public Command setVoltages(double rotor, double upgoer){
    return Commands.parallel(
      dyeRotor.setVoltage(rotor),
      upGoer.setVoltage(upgoer)
    );
  }

  public void periodic(){
    mech.update(dyeRotor.getPosition(), upGoer.getVelocity());
  }
}
