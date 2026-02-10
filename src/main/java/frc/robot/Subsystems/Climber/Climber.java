// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Climber;

import static edu.wpi.first.units.Units.Inches;

import frc.robot.Subsystems.Climber.ClimberTrolley.ClimberExtension;

/** Add your docs here. */
public class Climber {

    //first stage
    //second stage 

    //Same
        //behaviours
        //gearing
    //different
        // motor id
        // Range/limits?
        // name/description
        // invert/motor rotation direction
        
    ClimberExtension stage1 = new ClimberExtension(
        "Stage1", 19, false, Inches.of(6)
    );
    ClimberExtension stage2 = new ClimberExtension(
        "Stage2", 20, false, Inches.of(6)
    );

    public Climber(){

    }
    //helpful climber commands and groups to interface with things

    //extend to L1 climb position
    //goback down and climb

    //maybe: Climb to L2? L3?
}
