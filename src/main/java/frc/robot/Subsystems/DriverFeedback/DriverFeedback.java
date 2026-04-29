// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.DriverFeedback;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** Add your docs here. */
public class DriverFeedback {
    CommandXboxController driver;
    CommandXboxController operator;

    public DriverFeedback(CommandXboxController driver, CommandXboxController operator){
        this.driver = driver;
        this.operator = operator;
    }

    public Command shiftStart(){
        return Commands.parallel(
            new RumbleBoth(driver, 1.0, 0.5),
            new RumbleBoth(operator, 1.0, 0.5)
        );
    }

    public Command shiftEnd(){
        return Commands.parallel(
            new PulseBoth(driver, 1.0, 0.5, 2.0),
            new PulseBoth(operator, 1.0, 0.5, 2.0)
        );
    }

    public Command climberStowed(){
        return Commands.parallel(
            new PingPong(driver, 1.0, 0.5, 2.0),
            new PingPong(operator, 1.0, 0.5, 2.0)
        );
    }
}
