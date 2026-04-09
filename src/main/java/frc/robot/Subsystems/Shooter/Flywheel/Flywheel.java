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
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.TargetingSystem.TargetingSystem;

public class Flywheel extends SubsystemBase {

  static final double kGearing = 1.0;

  SparkFlex leaderMotor = new SparkFlex(17, MotorType.kBrushless);
  SparkFlex followerMotor = new SparkFlex(18, MotorType.kBrushless);

  private double targetRPM = 0.0;
  private double tolerance = 250.0;

  /** Creates a new Flywheel. */
  public Flywheel() {
    SparkBaseConfig followerConfig = getMotorConfig();
    followerConfig.follow(leaderMotor, true);

    leaderMotor.configure(getMotorConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    setDefaultCommand(Commands.waitSeconds(2).andThen(run(this::stop)));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("shooter/flywheel/targetrpm", targetRPM);
    SmartDashboard.putNumber("shooter/flywheel/rpm", leaderMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("shooter/flywheel/rpmSetpoint", leaderMotor.getClosedLoopController().getSetpoint());
    SmartDashboard.putNumber("shooter/flywheel/voltage", leaderMotor.getAppliedOutput()*leaderMotor.getBusVoltage());
    SmartDashboard.putNumber("shooter/flywheel/current", leaderMotor.getOutputCurrent());
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
      // leaderMotor.getClosedLoopController().setSetpoint(
      //   targetRPM, 
      //   SparkBase.ControlType.kVelocity
      // );
      leaderMotor.setVoltage(rpm.getAsDouble()*0.0024309 * 4000 / 5174.083984 * 2725 / 2516.0);
    });
  }

  public Command setRPM(Supplier<TargetingSystem.ShooterState> targetSupplier){
    return setRPM(()->targetSupplier.get().flywheelRPM,()->targetSupplier.get().flywheelRPM);
  }

  public boolean getOnTarget(){
    return MathUtil.isNear(targetRPM, leaderMotor.getEncoder().getVelocity(), tolerance*6);
  }

  private SparkBaseConfig getMotorConfig(){
    SparkBaseConfig config = new SparkFlexConfig();

    config
      .smartCurrentLimit(100)
      .inverted(true)
      .idleMode(IdleMode.kCoast)
      // .voltageCompensation(10.5)
      ;

    config.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.0)
      .i(0.000000)
      .d(0.0)
    .feedForward
      .kV(0.0024309 * 4000 / 5174.083984 * 2725 / 2516.0)
    ;

    config.encoder
      .uvwMeasurementPeriod(8)
      .quadratureAverageDepth(2)
      .quadratureMeasurementPeriod(8)
      .positionConversionFactor(kGearing)
      .velocityConversionFactor(kGearing) //Do NOT divide by 60, rpm is desired, not rps
    ;

    return config;
  }
}
