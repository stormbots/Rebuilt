// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Spindexer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.Spindexer.DyeRotor.DyeRotor;
import frc.robot.Subsystems.Spindexer.UpGoer.UpGoer;

public class Spindexer extends SubsystemBase{
  private DyeRotor dyeRotor = new DyeRotor();
  private UpGoer upGoer = new UpGoer();
  SpindexerVisual mech = new SpindexerVisual();
  private Trigger readyToFeed; 
  
  /** Creates a new Spindexer. */
  public Spindexer(Trigger readyToFeed){
      this.readyToFeed = readyToFeed;
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
      upGoer.feed(),
      new WaitCommand(0.25).andThen(dyeRotor.feed())
    );

    return Commands.repeatingSequence(
      stop().until(readyToFeed),
      feed.until(readyToFeed.negate())
    );
  }

  public Command feedToShooterForce(){
    var feed = Commands.parallel(
    upGoer.feed(),
    new WaitCommand(0.25).andThen(dyeRotor.feed())
    );

    return feed;
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
    SmartDashboard.putBoolean("spindexer/readyToShoot", readyToFeed.getAsBoolean());
  }
}
