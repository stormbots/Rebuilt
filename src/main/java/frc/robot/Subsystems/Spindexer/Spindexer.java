// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Spindexer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.stormbots.CRTAbsoluteEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase {
  SparkFlex motor = new SparkFlex(12, MotorType.kBrushless);
  SparkFlex motor2 = new SparkFlex(13, MotorType.kBrushless);
  /** Creates a new Spindexer. */
  public Spindexer() {
    SparkBaseConfig config = new SparkFlexConfig();

    // config.absoluteEncoder
    //   .positionConversionFactor(360.0)
    //   .inverted(true)
    // ;

    config.smartCurrentLimit(40)
    .inverted(false);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    config.inverted(true);
    
    motor2.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    CRTAbsoluteEncoder.getInstance().setEncoder2(motor.getAbsoluteEncoder());

    setDefaultCommand(run(()->{
      motor.stopMotor();
      motor2.stopMotor();
    }));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("/turret/e2", motor.getAbsoluteEncoder().getPosition());
  }

  public Command bruh(){
    return run(()->{
      motor.setVoltage(6);
      motor2.setVoltage(6);
    });
  }
}
