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

    /** Return the absolute rotation of the system, derived from the CRT process */
    public Angle getPosition(){
        double difference = encoder2.getPosition() - encoder1.getPosition();
        
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
        return Degrees.of(difference * kSlope);
    }

    public void sync(){
        relativeEncoder.setPosition(getPosition().in(Degrees));
    }

}
