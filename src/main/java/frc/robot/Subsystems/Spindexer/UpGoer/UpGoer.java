// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Spindexer.UpGoer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UpGoer extends SubsystemBase{
    SparkFlex motor = new SparkFlex(13, MotorType.kBrushless);
    UpGoerSim sim = new UpGoerSim(motor);

    public UpGoer(){
        var config = new SparkFlexConfig();

        var conversionfactor = 5.0;
        config.encoder.positionConversionFactor(1/conversionfactor);
        config.encoder.velocityConversionFactor(1/conversionfactor/60);

        config
        .idleMode(IdleMode.kCoast)
        .inverted(true)
        .smartCurrentLimit(20)
        ;

        // config.closedLoop
        // .p(0)
        // ;
        
        config.closedLoop.feedForward
        .kV(1/(7600.0/conversionfactor))
        ;

        motor.configure(config , ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        motor.getEncoder().setPosition(0);
        setDefaultCommand(stop());
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("UpGoer/Output", motor.getAppliedOutput());
        SmartDashboard.putNumber("UpGoer/Position",motor.getEncoder().getPosition());
        SmartDashboard.putNumber("UpGoer/Velocity",motor.getEncoder().getVelocity());
    }

    
    public void simulationPeriodic(){
        sim.update();
    }

    public Command setVelocity(double velocity){
        return run(()->{ 
            motor.getClosedLoopController().setSetpoint(velocity, ControlType.kVelocity);
        });
    }

    public Command stop(){
        return run(motor::stopMotor);
    }

    public Command feed(){
        return setVelocity(1000);
    }

    public Command unclog(){
        return setVelocity(1000);
    }

    public Command setVoltage(double volts){
        return run(()->motor.setVoltage(volts));
    }

    public double getVelocity(){
        return motor.getEncoder().getVelocity();
    }
        
}
