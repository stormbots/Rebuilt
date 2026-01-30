// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter.Feeder;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {

  SparkFlex leaderMotor = new SparkFlex(100, MotorType.kBrushless);
  SparkFlex followerMotor = new SparkFlex(100, MotorType.kBrushless);
  
  public static final double kGearing = 1.0;

  /** Creates a new Feeder. */
  public Feeder() {
    SparkBaseConfig followerConfig = getMotorConfig();
    followerConfig.follow(leaderMotor, true);

    leaderMotor.configure(getMotorConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private SparkBaseConfig getMotorConfig(){
    SparkBaseConfig config = new SparkFlexConfig();

    config
      .smartCurrentLimit(40)
      .inverted(false)
      .idleMode(IdleMode.kBrake);

    config.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.00025)
      .i(0.0)
      .d(0.0)
    .feedForward
      .kS(0.0)
      .kV(0.0)
    ;

    config.encoder
      .positionConversionFactor(kGearing)
      .velocityConversionFactor(kGearing) //Do NOT divide by 60, rpm is desired, not rps
    ;

    return config;
  }
}
