// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter.Turret;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.TargetingSystem.TargetingSystem;

public class Turret extends SubsystemBase {

  public static final double kTurretGearToothCount = 132;
  public static final double kGear1ToothCount = 20;
  public static final double kGear2ToothCount = 21;
  
  public static final double kGearing = (1.0 / 3.0) * (10.0 / 132.0);

  //How much in ONE direction, hence max range divided by 2
  public static final double kMinRotation = -45;
  public static final double kMaxRotation = 45;

  Angle targetPosition = Degrees.of(0);
  Angle tolerance = Degrees.of(3);

  SparkFlex motor = new SparkFlex(14, MotorType.kBrushless);
  TurretSim sim = new TurretSim(motor);

  /** Creates a new Turret. */
  public Turret() {

    motor.configure(getMotorConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    CRTAbsoluteEncoder.getInstance().setParams(kTurretGearToothCount, kGear1ToothCount, kGear2ToothCount, false);
    CRTAbsoluteEncoder.getInstance().setRelativeEncoder(motor.getEncoder());
    CRTAbsoluteEncoder.getInstance().setEncoder1(motor.getAbsoluteEncoder());

    // setDefaultCommand(run(()->motor.stopMotor()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("shooter/turret/applied", motor.getAppliedOutput());
    SmartDashboard.putNumber("shooter/turret/outc", motor.getOutputCurrent());
    SmartDashboard.putNumber("shooter/turret/target", targetPosition.in(Degrees));
    SmartDashboard.putNumber("shooter/turret/tolerance", tolerance.in(Degrees));
    SmartDashboard.putBoolean("shooter/turret/ontarget", getOnTarget());
    SmartDashboard.putNumber("shooter/turret/CRTpos", CRTAbsoluteEncoder.getInstance().getPosition().in(Degrees));
    SmartDashboard.putNumber("shooter/turret/e1", motor.getAbsoluteEncoder().getPosition());
    SmartDashboard.putNumber("shooter/turret/relPos", motor.getEncoder().getPosition());
  }

  /**
   * @return degrees
   */
  public Angle getAngle(){
    return Degrees.of(motor.getEncoder().getPosition());
  }

  public Command setAngle(Supplier<Angle> position, Supplier<Angle> tolerance){
    return run(()->{
      this.targetPosition = position.get();
      this.tolerance = tolerance.get();
      motor.getClosedLoopController().setSetpoint(
        targetPosition.in(Degrees), 
        ControlType.kPosition
      );
    });
  }

  public Command setAngle(Supplier<TargetingSystem.ShooterState> targetSupplier){
    return setAngle(()->targetSupplier.get().turretAngle, ()->targetSupplier.get().turretTolerance);
  }

  public boolean getOnTarget(){
    return MathUtil.isNear(targetPosition.in(Degrees), motor.getEncoder().getPosition(), tolerance.in(Degrees));
  }

  private SparkBaseConfig getMotorConfig(){
    SparkFlexConfig config = new SparkFlexConfig();

    config
      .smartCurrentLimit(30)
      .idleMode(IdleMode.kBrake)
      //giving positive power should turn the turret CCW
      .inverted(false)
    ;

    config.encoder
      .positionConversionFactor(kGearing * 360.0)
      .velocityConversionFactor(kGearing * 360.0 / 60.0)
    ;

    config.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.3 * 12 / 90.0)
    ;

    config.softLimit
      .forwardSoftLimit(kMaxRotation)
      .forwardSoftLimitEnabled(true)
      .reverseSoftLimit(kMinRotation)
      .reverseSoftLimitEnabled(true)
    ;

    config.absoluteEncoder
      .positionConversionFactor(360.0)
      .inverted(false)
    ;

    return config;
  }
}
