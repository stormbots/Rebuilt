// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.DriverFeedback;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PingPong extends Command {
  /** Creates a new RumbleBoth. */
  CommandXboxController controller;
  double startTime;
  double duration;
  double maxIntensity;
  double frequency;
  boolean finished;
  public PingPong(CommandXboxController controller, double duration, double maxIntensity, double frequency) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.controller = controller;
    this.duration = duration;
    this.maxIntensity = maxIntensity;
    if (maxIntensity > 1.0){
      maxIntensity = 1.0;
    }
    if (maxIntensity < 0.0){
      maxIntensity = 0.0;
    }
    this.frequency = frequency;
    if (frequency == 0.0){
      frequency = 1.0;
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

    double leftIntensity = (maxIntensity/2)*Math.sin((2*Math.PI*elapsedTime*frequency)-(Math.PI/2))+(maxIntensity/2);
    double rightIntensity = (maxIntensity/2)*Math.cos(2*Math.PI*elapsedTime*frequency)+(maxIntensity/2);

    controller.setRumble(RumbleType.kLeftRumble, leftIntensity);
    controller.setRumble(RumbleType.kRightRumble, rightIntensity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    controller.setRumble(RumbleType.kBothRumble, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
