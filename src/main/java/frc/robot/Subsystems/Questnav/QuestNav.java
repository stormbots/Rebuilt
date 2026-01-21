// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Questnav;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Photonvision.Photonvision;
import frc.robot.Subsystems.Swerve.Swerve;

public class QuestNav extends SubsystemBase {
  /** Creates a new QuestNav. */
  public QuestNav(Swerve swerve, Photonvision photonvision) {
    //Photonvision may not be needed here, but including it anyway for now
    //TODO Auto-generated constructor stub
}

@Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
