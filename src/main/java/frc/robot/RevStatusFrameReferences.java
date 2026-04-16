// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/** This only exists because setting these. */
public class RevStatusFrameReferences {
    // Should be lowered on leader motors to allow followers to more reliably catch the signal 
    // Increase on followers, since we never care about them */
    // public static PeriodicFrame kPeriodicStatusFrame = PeriodicFrame.kStatus0;

    // /** 0: (10ms) applied output, faults, follower status ; 
    
    // /** 1: (10ms) Motor velocity, temperature, input bus voltage, motor current */
    // public static PeriodicFrame kVelocityCurrent = PeriodicFrame.kStatus1;
    
    // /** 2: (20ms) Motor position */
    // public static PeriodicFrame kPosition = PeriodicFrame.kStatus2;

    // /** 4: (20ms) Alternate Encoder inputs ; Never really used */
    // public static PeriodicFrame kAltEncodPeriodicFrame = PeriodicFrame.kStatus4;
    
    // /** 5: (20ms) Absolute Encoder inputs */
    // public static PeriodicFrame kAbsEncoderPosition = PeriodicFrame.kStatus5;
    
    // /** 6: (20ms) Absolute Encoder inputs */
    // public static PeriodicFrame kAbsEncoderVelocity = PeriodicFrame.kStatus5;


    /**
    https://docs.revrobotics.com/brushless/spark-max/control-interfaces
    //For your copy-pasting joy, using initial default values

    config.apply(new SignalsConfig()
        .appliedOutputPeriodMs(10) //frame 0 : Applied output, faults ;
        .primaryEncoderVelocityPeriodMs(20) //frame 1 : Velocity, temp, input voltage, stator current
        .primaryEncoderPositionPeriodMs(20) // frame 2 : Motor position
        .absoluteEncoderPositionPeriodMs(20) //frame 5 
        // .absoluteEncoderVelocityPeriodMs(20) //frame 6
    );


    */
}
