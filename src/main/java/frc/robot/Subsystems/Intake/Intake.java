// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.HopperSensors.HopperSensors;
import frc.robot.Subsystems.Intake.IntakeExtension.IntakeExtension;
import frc.robot.Subsystems.Intake.OuterRollers.OuterRollers;
import frc.robot.Subsystems.Intake.Rollers.Rollers;

/** Add your docs here. */
public class Intake {
    private IntakeExtension intakeExtension = new IntakeExtension();
    private Rollers rollers = new Rollers();
    private OuterRollers outerrollers = new OuterRollers();

    /** Boolean to check if deployed. Mostly to facilitate Fuel 
     * simulation, and should be replaced with a Trigger checking the
     * deployment angle eventually.
     */ 
    public boolean isDeployed = false;


    /** Pause intaking when full, resume intaking when space available.*/
    public Command smartIntake(){
        //TODO: Quickly cobbled to test sim
        return Commands.either(
            Commands.run(()->isDeployed=true).until(HopperSensors.getInstance().isFull),
            Commands.run(()->isDeployed=false).until(HopperSensors.getInstance().isNotFull),
            HopperSensors.getInstance().isNotFull
        )
        .repeatedly()
        .finallyDo(()->isDeployed=false)
        ;
    }

}
