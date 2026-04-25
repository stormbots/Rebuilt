// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.DriverFeedback;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RumbleBoth extends Command {
  /** Creates a new RumbleBoth. */
  CommandXboxController controller;
  double startTime;
  double duration;
  double intensity;
  boolean finished;
  public RumbleBoth(CommandXboxController controller, double duration, double intensity) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.controller = controller;
    this.duration = duration;
    this.intensity = intensity;
    if (intensity > 1.0){
      intensity = 1.0;
    }
    if (intensity < 0.0){
      intensity = 0.0;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var currentTime = Timer.getFPGATimestamp();
    var elapsedTime = currentTime-startTime;
    if (elapsedTime>duration){
      finished = true;
    }
    controller.getHID().setRumble(RumbleType.kBothRumble, intensity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
