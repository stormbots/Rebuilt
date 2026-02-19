// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Spindexer.DyeRotor;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DyeRotor extends SubsystemBase{
    SparkFlex motor = new SparkFlex(12, MotorType.kBrushless);
    DyeRotorSim sim = new DyeRotorSim(motor);
    
    public DyeRotor(){
        var config = new SparkFlexConfig();

        var conversionfactor = 1.0;
        config.encoder.positionConversionFactor(1/conversionfactor);
        config.encoder.velocityConversionFactor(1/conversionfactor/60);

        config
        .idleMode(IdleMode.kCoast)
        .inverted(false)
        .smartCurrentLimit(30)
        ;

        // config.closedLoop
        // .p(0)
        // ;
        config.closedLoop.feedForward
        .kV(12/7600.0)
        ;

        motor.configure(config , ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        setDefaultCommand(stop());
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("DyeRotar/Output", motor.getAppliedOutput());
    }

    @Override
    public void simulationPeriodic(){
        sim.update();
    }

    public Command setVelocity (double targetVelocity){
        return run(()->{ 
            //TODO Change this from voltage to velocity,  very important!!!!!!!!!!
            motor.getClosedLoopController().setSetpoint(targetVelocity, ControlType.kVoltage);
        });
    }

    public void setVoltage(double volt){
        motor.setVoltage(volt);
    }

    public Command stop(){
        return setVelocity(0);
    }

    public Command spin(){
        return setVelocity(5);
    }

    public Command load(){
        return setVelocity(5);
    }

    public Command spinBackwards(){
        return setVelocity(-5);
    }

    public AngularVelocity getVelocity(){
        return DegreesPerSecond.of(motor.getEncoder().getVelocity());
    }

    public double getPosition(){
        return motor.getEncoder().getPosition();
    }

    public double getCurrent(){
        return motor.getOutputCurrent();
    }
    
}
