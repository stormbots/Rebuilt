// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.stormbots;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class CRTAbsoluteEncoder {

    private static CRTAbsoluteEncoder instance;
     
    private double kTurretGearToothCount;
    private double kGear1ToothCount;
    private double kGear2ToothCount;
    private double kSlope;
    private boolean kInverted;

    private RelativeEncoder relativeEncoder;
    private SparkAbsoluteEncoder encoder1;
    private SparkAbsoluteEncoder encoder2;

    private CRTAbsoluteEncoder(){}

    public static CRTAbsoluteEncoder getInstance(){
        if(instance==null){
            instance = new CRTAbsoluteEncoder();
        }

        return instance;
    }

    /** Assumes both absolute encoders are zero at turret angle of zero and increase in the same direction */
    public void setParams(
        double turretGearTeeth, 
        double gear1Teeth, 
        double gear2Teeth, 
        boolean inverted
    ){
        kTurretGearToothCount = turretGearTeeth;
        kGear1ToothCount = gear1Teeth;
        kGear2ToothCount = gear2Teeth;

        kSlope = (kGear2ToothCount * kGear1ToothCount)
            / ((kGear1ToothCount - kGear2ToothCount) * kTurretGearToothCount);


        kInverted = inverted;
    }

    public void setRelativeEncoder(RelativeEncoder encoder){
        this.relativeEncoder = encoder;
    }

    public void setEncoder1(SparkAbsoluteEncoder encoder){
        this.encoder1 = encoder;
    }

    public void setEncoder2(SparkAbsoluteEncoder encoder){
        this.encoder2 = encoder;
    }

    public boolean isReady(){
        if(encoder1==null || encoder2==null) return false;
        return true;
    }

    /** Return the absolute rotation of the system, derived from the CRT process */
    public Angle getPosition(){
        if(isReady()==false) return Degrees.of(-180);

        double difference = encoder2.getPosition() - encoder1.getPosition();
        //Slight scaling issue that needs to be resolved
        double kFudgefactor = 180.0/192.0;
        
        if(kInverted){
            difference*=-1;
        }
        
        if (difference < -180){
            difference += 360;
        }
        else if(difference > 180){
            difference -= 360;
        }

        SmartDashboard.putNumber("shooter/turret/difference", difference);
        
        //Difference increases linearly with turret angle
        //Multiply by slope to get turret angle from difference
        return Degrees.of(difference * kSlope * kFudgefactor);
    }

    public void sync(){
        //Assume the bot was set correctly.
        relativeEncoder.setPosition(-180);

        if(isReady()==false) return;

        var crt = getPosition();
        if(crt.isNear(Degrees.of(-180), Degrees.of(-10))) return;

        
        //If not, fall back to the CRT as a safety measure
        // relativeEncoder.setPosition(getPosition().in(Degrees));
    }

}
