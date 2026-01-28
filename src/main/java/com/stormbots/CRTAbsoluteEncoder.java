// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.stormbots;

import com.revrobotics.spark.SparkAbsoluteEncoder;

/** Add your docs here. */
public class CRTAbsoluteEncoder {
    private final double TURRET_GEAR_TOOTH_COUNT;
    private final double GEAR_1_TOOTH_COUNT;
    private final double GEAR_2_TOOTH_COUNT;
    private final double SLOPE;

    private final boolean INVERTED;


    /**
     * Assumes both absolute encoders are zero at turret angle of zero and increase in the same direction
     */
    public CRTAbsoluteEncoder(
        double turretGearTeeth, 
        double gear1Teeth, 
        double gear2Teeth, 
        boolean inverted
    ){
        TURRET_GEAR_TOOTH_COUNT = turretGearTeeth;
        GEAR_1_TOOTH_COUNT = gear1Teeth;
        GEAR_2_TOOTH_COUNT = gear2Teeth;

        SLOPE = (GEAR_2_TOOTH_COUNT * GEAR_1_TOOTH_COUNT)
            / ((GEAR_1_TOOTH_COUNT - GEAR_2_TOOTH_COUNT) * TURRET_GEAR_TOOTH_COUNT);


        INVERTED = inverted;

    }


    /**
     * @param e1 degrees pls
     * @param e2 degrees pls
     * @return turret angle in degrees
     */
    public double getAngle(double e1, double e2){
        double difference = e2 - e1;

        if(INVERTED){
            difference*=-1;
        }

        if (difference < -180){
            difference += 360;
        }
        else if(difference > 180){
            difference -= 360;
        }

        //Difference increases linearly with turret angle
        //Multiply by slope to get turret angle from difference
        return difference * SLOPE;
    }

}
