// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter.Turret;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.stormbots.CRTAbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

  public static final double kTurretGearToothCount = 100;
  public static final double kGear1ToothCount = 16;
  public static final double kGear2ToothCount = 17;
  
  public static final double kGearing = 1.0;

  //How much in ONE direction, hence max range divided by 2
  public static final double kMaxRotation = 540.0 / 2.0;

  Angle targetPosition = Degrees.of(0);
  Angle tolerance = Degrees.of(3);


  SparkFlex motor = new SparkFlex(99, MotorType.kBrushless);

  /** Creates a new Turret. */
  public Turret() {

    motor.configure(getMotorConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    CRTAbsoluteEncoder.getInstance().setParams(kTurretGearToothCount, kGear1ToothCount, kGear2ToothCount, true);
    CRTAbsoluteEncoder.getInstance().setRelativeEncoder(motor.getEncoder());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * @return degrees
   */
  public Angle getAngle(){
    return Degrees.of(motor.getEncoder().getPosition());
  }

  private void setAngle(Angle position, Angle tolerance) {
    this.targetPosition = position;
    this.tolerance = tolerance;
    motor.getClosedLoopController().setSetpoint(
      position.in(Degrees), 
      ControlType.kPosition
    );
  }

  public Command setAngleCommand(Angle position, Angle tolerance){
    return run(()->setAngle(position, tolerance));
  }

  public boolean getOnTarget(){
    return MathUtil.isNear(targetPosition.in(Degrees), motor.getEncoder().getPosition(), tolerance.in(Degrees));
  }

  private SparkBaseConfig getMotorConfig(){
    SparkFlexConfig config = new SparkFlexConfig();

    config
      .smartCurrentLimit(40)
      .idleMode(IdleMode.kBrake)
      //giving positive power should turn the turret CCW
      .inverted(false)
    ;

    config.encoder
      .positionConversionFactor(kGearing)
      .velocityConversionFactor(kGearing / 60.0)
      //Turning the turret CCW should increase position
      .inverted(false)
    ;

    config.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.0)
    ;

    config.softLimit
      .forwardSoftLimit(kMaxRotation)
      .forwardSoftLimitEnabled(true)
      .reverseSoftLimit(-kMaxRotation)
      .reverseSoftLimitEnabled(true)
    ;

    return config;
  }
}
