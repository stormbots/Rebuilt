// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Spindexer.DyeRotor;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DyeRotor extends SubsystemBase{
    SparkFlex motor = new SparkFlex(12, MotorType.kBrushless);
    DyeRotorSim sim = new DyeRotorSim(motor);
    int maxCurrent = 40;
    int lowCurrentLimit = 25;
    
    public DyeRotor(){
        var config = new SparkFlexConfig();

        var conversionfactor = 57.0;
        config.encoder.positionConversionFactor(1/conversionfactor);
        config.encoder.velocityConversionFactor(1/conversionfactor/60);

        config
        .idleMode(IdleMode.kCoast)
        .inverted(false)
        .smartCurrentLimit(maxCurrent)
        ;

        // config.closedLoop
        // .p(0)
        // ;
        config.closedLoop.feedForward
        .kV(12/(7600.0/conversionfactor)*3)
        ;

        motor.configure(config , ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        motor.getEncoder().setPosition(0);
        setDefaultCommand(stop());
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("DyeRotor/Output", motor.getAppliedOutput());
        SmartDashboard.putNumber("DyeRotor/Position",motor.getEncoder().getPosition());
        SmartDashboard.putNumber("DyeRotor/Velocity",motor.getEncoder().getVelocity());
    }

    @Override
    public void simulationPeriodic(){
        sim.update();
    }

    public Command setVelocity(double targetVelocity){
        return run(()->{ 
            motor.getClosedLoopController().setSetpoint(targetVelocity, ControlType.kVelocity);
        });
    }

    public Command setVoltage(double volt){
        return run(() ->{
            motor.setVoltage(volt);
        });
    }

    public Command stop(){
        return run(motor::stopMotor);
    }

    public Command feed(){
        //return setVelocity(2);
        return setVoltage(7);
    }

    public Command intake(){
        return setVoltage(8).beforeStarting(runOnce(()-> setCurrentLimits(lowCurrentLimit))).finallyDo(()->setCurrentLimits(maxCurrent));
    }

    public Command unclog(){
        return setVoltage(-8);
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

    private void setCurrentLimits(int amps){
        var config = new SparkFlexConfig();
        motor.configureAsync(config.smartCurrentLimit(amps), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
}
