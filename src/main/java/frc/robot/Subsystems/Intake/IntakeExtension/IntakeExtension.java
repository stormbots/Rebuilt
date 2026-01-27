// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake.IntakeExtension;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class IntakeExtension extends SubsystemBase {
  SparkMax motor = new SparkMax(34, MotorType.kBrushless);

  /** Creates a new IntakeExtension. */
  public IntakeExtension() {
     var config = new  SparkFlexConfig();
    double factor = 1; //convert to surface speed of roller
    config.encoder
    .positionConversionFactor(factor)
    .velocityConversionFactor(factor/60);

    var absfactor=1;
    config.absoluteEncoder
    .inverted(false)
    .positionConversionFactor(absfactor)
    .velocityConversionFactor(absfactor/60)
    ;

    
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
  

    new Trigger(DriverStation::isEnabled)
    .onTrue(setIdleMode(IdleMode.kCoast))
    .onFalse(setIdleMode(IdleMode.kBrake))
    ;
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command up(){
    return run(()->{
      motor
      .getClosedLoopController()
      .setSetpoint(90, ControlType.kMAXMotionPositionControl);
    });
  }

  public Command down(){
    return run(()->{
      motor
      .getClosedLoopController()
      .setSetpoint(0, ControlType.kMAXMotionPositionControl);
    });
    
  }



  private Command setIdleMode(IdleMode mode){
    return Commands.runOnce(()->{
      var config = new SparkFlexConfig().idleMode(mode);
      motor.configure(config,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters);
    });
  }
}
