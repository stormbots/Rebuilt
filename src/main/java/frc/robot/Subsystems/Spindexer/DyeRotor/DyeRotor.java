// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Spindexer.DyeRotor;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DyeRotor extends SubsystemBase{
    SparkFlex motor = new SparkFlex(12, MotorType.kBrushless);
    DyeRotorSim sim = new DyeRotorSim(motor);

    RelativeEncoder encoder = motor.getEncoder();
    
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

        config.closedLoop
        .p(0)
        ;
        config.closedLoop.feedForward
        .kV(12/7600.0)
        ;

        motor.configure(config , ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
        return Commands.run(()->{ 
            motor.getClosedLoopController().setSetpoint(targetVelocity, ControlType.kVelocity);
        });
    }

    public Command stop(){
        return setVelocity(0);
    }

    public Command spin(){
        return setVelocity(10);
    }

    public Command spinBackwards(){
        return setVelocity(-10);
    }

    public double getVelocity(){
        return encoder.getVelocity();
    }

    public Angle getPosition(){
        return Degrees.of(encoder.getPosition());
    }
    
}
