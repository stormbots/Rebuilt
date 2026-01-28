// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Intake.IntakeExtension.IntakeExtension;
import frc.robot.Subsystems.Intake.Rollers.Rollers;

/** Add your docs here. */
public class Intake {
    private IntakeExtension intakeExtension = new IntakeExtension();
    private Rollers rollers = new Rollers();

    //TODO: Create a lot of useful commands here

    // Interface: 
    // intake
    // eject / get rid of stuck things


    public Command intake(){
        return Commands.parallel(
            rollers.intake(),
            intakeExtension.down()
        );
    };

    public Command eject(){
        return Commands.parallel(
            // rollers.eject()
        );
    }

}
