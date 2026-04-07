// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Climber;

import static edu.wpi.first.units.Units.Inches;

import com.stormbots.LaserCanWrapper;

import frc.robot.Subsystems.Swerve.Swerve.SwerveInputs;

/** Add your docs here. */
public class Rangefinders {
  public LaserCanWrapper left = new LaserCanWrapper(2).configureShortRange().setThreshhold(Inches.of(24));
  public LaserCanWrapper right = new LaserCanWrapper(3).configureShortRange().setThreshhold(Inches.of(24));
  SwerveInputs swerveInputs = new SwerveInputs();

  /**
   * @param facingAngleDegrees Valid for +90 or -90 degrees
   * @return
   */
  public SwerveInputs generateInputs(double facingAngleDegrees) {
    swerveInputs.clear();
    if (left.isBreakBeamTripped.getAsBoolean() && right.isBreakBeamTripped.getAsBoolean()) {
      swerveInputs.ty = 0.05;
    } else if (left.isBreakBeamTripped.getAsBoolean()) {
      swerveInputs.tx = -0.05;
    } else if (right.isBreakBeamTripped.getAsBoolean()) {
      swerveInputs.tx = 0.05;
    }

    if (facingAngleDegrees == 90) {
      // flip outputs for going the other field direction
      swerveInputs.tx *= -1;
      swerveInputs.ty *= -1;
    }
    return swerveInputs;
  }

  public boolean isLinedUpL1() {
    var leftOk = left.getDistanceOptional().orElse(Inches.of(12)).lt(Inches.of(5));
    var rightOk = right.getDistanceOptional().orElse(Inches.of(12)).lt(Inches.of(5));
    return leftOk && rightOk;
  }
}
