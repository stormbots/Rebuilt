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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpindexerButBad extends SubsystemBase {
  public SparkFlex dyerotor = new SparkFlex(12,MotorType.kBrushless);
  public SparkFlex upgoer = new SparkFlex(13,MotorType.kBrushless);
  
  /** Creates a new Spindexer. */
  public SpindexerButBad() {
    SparkBaseConfig config = new SparkFlexConfig();

    config
    .smartCurrentLimit(30)
    .inverted(false)
    .idleMode(IdleMode.kCoast)
    ;

    dyerotor.configureAsync(config,ResetMode.kResetSafeParameters,PersistMode.kNoPersistParameters);


    config
    .inverted(true)
    ;

    upgoer.configureAsync(config,ResetMode.kResetSafeParameters,PersistMode.kNoPersistParameters);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private Command rotate(double percent){
    return runEnd(()->dyerotor.set(percent), dyerotor::stopMotor);
  }

  private Command feedUpward(double percent){
    return runEnd(()->upgoer.set(percent), upgoer::stopMotor);
  }

  public Command feedToShooter(){
    return Commands.parallel(
      rotate(0.5),
      feedUpward(0.5)
    );
  }

  public Command spinToLoad(){
    return Commands.parallel(
      rotate(0.25)
    );
  }
}
