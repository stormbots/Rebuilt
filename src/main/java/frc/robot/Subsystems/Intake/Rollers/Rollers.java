// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake.Rollers;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase {

  SparkMax motor = new SparkMax(35, MotorType.kBrushless);

  /** Creates a new Rollers. */
  public Rollers() {
    //configure the motor
    var config = new  SparkFlexConfig();
    double factor = 1; //convert to surface speed of roller
    config.encoder
    .positionConversionFactor(factor)
    .velocityConversionFactor(factor/60);
    
    config.closedLoop.feedForward
    .sva(0, 12/6000.0/factor, 0);

    config.closedLoop
    .p(0);

    config.closedLoop.maxMotion
    .maxAcceleration(20)
    .cruiseVelocity(6000/60);

    config
    .idleMode(IdleMode.kCoast)
    .inverted(false)
    .smartCurrentLimit(20)
    .voltageCompensation(11)
    ;

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command intake(){
  return run(()->{
      motor
      .getClosedLoopController()
      .setSetpoint(5, ControlType.kMAXMotionVelocityControl);
    });

  }


}
