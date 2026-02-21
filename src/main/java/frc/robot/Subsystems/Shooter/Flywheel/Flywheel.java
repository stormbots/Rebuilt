// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter.Flywheel;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.stormbots.Clamp;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.TargetingSystem.TargetingSystem;

public class Flywheel extends SubsystemBase {

  static final double kGearing = 1.0;

  SparkFlex leaderMotor = new SparkFlex(17, MotorType.kBrushless);
  SparkFlex followerMotor = new SparkFlex(18, MotorType.kBrushless);

  private double targetRPM = 0.0;
  private double tolerance = 300.0;

  /** Creates a new Flywheel. */
  public Flywheel() {
    SparkBaseConfig followerConfig = getMotorConfig();
    followerConfig.follow(leaderMotor, true);

    leaderMotor.configure(getMotorConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("shooter/flywheel/targetrpm", targetRPM);
    SmartDashboard.putNumber("shooter/flywheel/rpm", leaderMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("shooter/flywheel/rpmSetpoint", leaderMotor.getClosedLoopController().getSetpoint());
    SmartDashboard.putNumber("shooter/flywheel/rotations", leaderMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("shooter/flywheel/voltage", leaderMotor.getAppliedOutput()*leaderMotor.getBusVoltage());
  }

  public double getRPM(){
    return leaderMotor.getEncoder().getVelocity();
  }
  
  private void stop(){
    this.targetRPM = 0;
    this.tolerance = 300; 
    leaderMotor.stopMotor();
  }

  public Command setRPM(DoubleSupplier rpm, DoubleSupplier tolerance){
    return run(()->{
      this.targetRPM = rpm.getAsDouble();
      this.tolerance = tolerance.getAsDouble();
      leaderMotor.getClosedLoopController().setSetpoint(
        targetRPM, 
        SparkBase.ControlType.kVelocity
      );
    });

  }

  public Command setRPM(Supplier<TargetingSystem.ShooterState> targetSupplier){
    return setRPM(()->targetSupplier.get().flywheelRPM,()->targetSupplier.get().flywheelRPM);
  }

  //Remove this once not needed
  public Command setVoltageCommand(double volts){
    return run(()->leaderMotor.setVoltage(volts));
  }

  public boolean getOnTarget(){
    return MathUtil.isNear(targetRPM, leaderMotor.getEncoder().getVelocity(), tolerance);
  }

  private SparkBaseConfig getMotorConfig(){
    SparkBaseConfig config = new SparkFlexConfig();

    config
      .smartCurrentLimit(100)
      .inverted(false)
      .idleMode(IdleMode.kCoast);

    config.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.00025)
      .i(0.0)
      .d(0.0)
    .feedForward
      .kS(0.0) //0.22
      .kV(0.0024309)
    ;

    config.encoder
      .positionConversionFactor(kGearing)
      .velocityConversionFactor(kGearing) //Do NOT divide by 60, rpm is desired, not rps
    ;

    return config;
  }

}
