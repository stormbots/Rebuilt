// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Climber;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.Climber.ClimberExtension.ClimberExtension;

/** Add your docs here. */
public class Climber extends SubsystemBase {
    public static Distance kStage1Range = Inches.of(6);
    public static Distance kStage2Range = Inches.of(20);

    ClimberExtension stage1 = new ClimberExtension(
        "Stage1", 19, true, kStage1Range
    );
    ClimberExtension stage2 = new ClimberExtension(
        "Stage2", 20, true, kStage2Range
    );

    public ClimberVisual visual = new ClimberVisual();

    public Climber(){


        // new Trigger(()->stage1.isHomed() && stage2.isHomed())
        // .whileTrue(
        //     stage2.setHeight(kStage2Range.minus(Inches.of(3)))
        //     .withName("PresetStage2")
        // );
    }

    @Override
    public void periodic(){
        visual.update(stage1.getHeight(), stage2.getHeight());
    }


    public Command goHome(){
        return Commands.parallel(
            stage1.goHome(),
            stage2.goHome()
        );
    }

    public Command setStage1Voltage(double voltage){
        return stage1.setVoltage(voltage);
    }
    public Command setStage2Voltage(double voltage){
        return stage2.setVoltage(voltage);
    }

    public Command prepareForClimbL1(){
        //move stage 1 up to a specific height
        return stage1.setHeight(Inches.of(6))
        .finallyDo(stage1::stopMotor)
        .withName("PrepareToClimb");
    }

    public Command climbL1(){
        return stage1.setHeight(Inches.of(0))
        .finallyDo(stage1::stopMotor)
        .withName("Climb");
    }
}
