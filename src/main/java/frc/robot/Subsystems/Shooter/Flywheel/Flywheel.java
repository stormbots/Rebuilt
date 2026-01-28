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
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {

  static final double kGEARING = (18.0) / (24.0);

  SparkFlex flywheelMotor1 = new SparkFlex(13, MotorType.kBrushless);
  SparkFlex flywheelMotor2 = new SparkFlex(9, MotorType.kBrushless);

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
    SparkFlexConfig config = new SparkFlexConfig();
    config.inverted(false)
    .idleMode(IdleMode.kCoast);
    config.closedLoop
    .p(0.00025)
    .i(0.0)
    .d(0.0)
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .feedForward
    .kS(0.0)//0.22
    .kV(0.0024309)
    // .kA(0.0)
    ;

    config.encoder
      .positionConversionFactor(kGEARING)
      .velocityConversionFactor(kGEARING) //rpm, not rps
    ;

    SparkFlexConfig followerConfig = new SparkFlexConfig();
    followerConfig.follow(flywheelMotor1, true);
    followerConfig.idleMode(IdleMode.kCoast);
    followerConfig.encoder
      .positionConversionFactor(kGEARING)
      .velocityConversionFactor(kGEARING) //rpm, not rps
    ;

    flywheelMotor1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    flywheelMotor2.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    systemState = handleStateTransitions();
    applyStates();

    SmartDashboard.putNumber("shooter/flywheel/targetrpm", targetRPM);
    SmartDashboard.putNumber("shooter/flywheel/rpm", flywheelMotor1.getEncoder().getVelocity());
    SmartDashboard.putNumber("shooter/flywheel/rpmSetpoint", flywheelMotor1.getClosedLoopController().getSetpoint());
    SmartDashboard.putNumber("shooter/flywheel/rotations", flywheelMotor1.getEncoder().getPosition());
    SmartDashboard.putNumber("shooter/flywheel/voltage", flywheelMotor1.getAppliedOutput()*flywheelMotor1.getBusVoltage());
  }
  
  private void setRPM(){
    flywheelMotor1.getClosedLoopController().setSetpoint(
      targetRPM, 
      SparkBase.ControlType.kVelocity
    );
  }

  private void stop(){
    flywheelMotor1.stopMotor();
  }

  public SystemState handleStateTransitions(){
    switch (wantedState){
    case SETRPM:
      return SystemState.SETRPM;
    case STOP:
      return SystemState.STOP;
    }

    return SystemState.STOP;
  }
  
  public void applyStates(){
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


  public void SetWantedState(WantedState wantedState){
    this.wantedState = wantedState;
  }

  public void SetWantedState(WantedState wantedState, double targetRPM){
    this.wantedState = wantedState;
    this.targetRPM = targetRPM;
  }

}
