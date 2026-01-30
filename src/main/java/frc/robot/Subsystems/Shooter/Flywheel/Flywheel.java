// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter.Flywheel;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {

  static final double kGearing = (18.0) / (24.0);

  SparkFlex leaderMotor = new SparkFlex(13, MotorType.kBrushless);
  SparkFlex followerMotor = new SparkFlex(9, MotorType.kBrushless);

  private double targetRPM = 0.0;

  public static enum WantedState{
    SETRPM,
    STOP
  }
  
  public static enum SystemState{
    SETRPM,
    STOP
  }

  private WantedState previousWantedState = WantedState.STOP;
  private WantedState wantedState = WantedState.STOP;
  private SystemState systemState = SystemState.STOP;

  /** Creates a new Flywheel. */
  public Flywheel() {
    SparkBaseConfig followerConfig = getMotorConfig();
    followerConfig.follow(leaderMotor, true);

    leaderMotor.configure(getMotorConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    systemState = handleStateTransitions();
    applyStates();

    SmartDashboard.putNumber("shooter/flywheel/targetrpm", targetRPM);
    SmartDashboard.putNumber("shooter/flywheel/rpm", leaderMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("shooter/flywheel/rpmSetpoint", leaderMotor.getClosedLoopController().getSetpoint());
    SmartDashboard.putNumber("shooter/flywheel/rotations", leaderMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("shooter/flywheel/voltage", leaderMotor.getAppliedOutput()*leaderMotor.getBusVoltage());
  }
  
  private void setRPM(){
    leaderMotor.getClosedLoopController().setSetpoint(
      targetRPM, 
      SparkBase.ControlType.kVelocity
    );
  }

  private void stop(){
    leaderMotor.stopMotor();
  }

  private SystemState handleStateTransitions(){
    switch (wantedState){
    case SETRPM:
      return SystemState.SETRPM;
    case STOP:
      return SystemState.STOP;
    }

    return SystemState.STOP;
  }
  
  private void applyStates(){
    switch(systemState){
      case SETRPM:
        setRPM();
        break;
      case STOP:
        targetRPM=0.0;
        stop();
        break;
    }
  }

  public void setWantedState(WantedState wantedState){
    this.wantedState = wantedState;
  }

  public void setWantedState(WantedState wantedState, double targetRPM){
    this.wantedState = wantedState;
    this.targetRPM = targetRPM;
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
