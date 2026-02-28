// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake.IntakeExtension;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.stormbots.CRTAbsoluteEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeExtension extends SubsystemBase {

  SparkMax motor = new SparkMax(9, MotorType.kBrushless);
  /** Creates a new IntakeExtension. */
  public IntakeExtension() {
    SparkBaseConfig config = new SparkMaxConfig();

    config.absoluteEncoder
      .positionConversionFactor(360.0)
      .inverted(false)
    ;

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    CRTAbsoluteEncoder.getInstance().setEncoder2(motor.getAbsoluteEncoder());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("shooter/turret/e2", motor.getAbsoluteEncoder().getPosition());
  }
}
