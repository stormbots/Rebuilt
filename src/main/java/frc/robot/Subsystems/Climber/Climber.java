// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Climber;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Climber.ClimberTrolley.ClimberExtension;

/** Add your docs here. */
public class Climber {
        
    ClimberExtension stage1 = new ClimberExtension(
        "Stage1", 19, true, Inches.of(6)
    );
    ClimberExtension stage2 = new ClimberExtension(
        "Stage2", 20, true, Inches.of(6)
    );

    public Climber(){

    }
    //helpful climber commands and groups to interface with things

    //extend to L1 climb position
    //goback down and climb

    //maybe: Climb to L2? L3?

    public Command goHome(){
        return Commands.parallel(
            stage1.goHome(),
            stage2.goHome()
        );
    }

}
