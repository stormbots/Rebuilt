// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.annotation.ElementType;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class ShiftTracking {
    public static Trigger canShoot;
    public ShiftTracking(){
        canShoot = new Trigger(this::isScoringShift);
    }

    private boolean didWeWinAuto(){
        if(DriverStation.getGameSpecificMessage().equals("R") && DriverStation.getAlliance().get().equals((Alliance.Red))){
            return true;
        }
        else if(DriverStation.getGameSpecificMessage().equals("B") && DriverStation.getAlliance().get().equals((Alliance.Blue))){
            return true;
        }
        else{
            return false;
        }
    }

    private Boolean isScoringShift(){
        double time = Timer.getMatchTime();
        if(DriverStation.isTeleopEnabled()){
            //endgame period
            if (time <= 30){
                return true;
            }
            //transition shift
            else if (time <= 130){
                return true;
            }
            //alliance shifts
            else if (didWeWinAuto()){
                return (((time < 105) && (time > 80)) || ((time < 55) && (time > 30)));
            }
            else{
                return !(((time < 105) && (time > 80)) ||((time < 55) && (time > 30)));
            }
        }
        else{
            return false;
        }
    }
    
}
