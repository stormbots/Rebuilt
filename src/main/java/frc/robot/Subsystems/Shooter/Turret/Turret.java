// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter.Turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.stormbots.CRTAbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.TargetingSystem.TargetingSystem;

public class Turret extends SubsystemBase {

  public static final double kTurretGearToothCount = 132;
  public static final double kGear1ToothCount = 20;
  public static final double kGear2ToothCount = 21;

  public static final double kGearing = (1.0 / 3.0) * (10.0 / 132.0) * (177.0 / 198.0) / (176.606 / 180.0);

  // How much in ONE direction, hence max range divided by 2
  // public static final double kMinRotation = -310.0;
  // public static final double kMaxRotation = 12.5;
  public static final double kMinRotation = -305.0;
  public static final double kMaxRotation = 10.0;
  private double outPut = 3 / 12.0 * 1.10;

  Angle targetPosition = Degrees.of(0);
  Angle tolerance = Degrees.of(3);

  //TODO: TUNE: some really weird overshoot problems
  //1. it hits the target, stops, then overshoots seemingly
  //2. with the wrapping done in the start of the startrun command for the trap profile (i assume)
  // it will sometimes pass the boundary and then try to wrap out of our range
  // I assume this is because the statement is >20 -=360, and since the trap profile is especially vulnerable to overshooting
  //it gets over 20, resets the current state, and tries to wrap the wrong way
  // does this also speak to our defaults for setAngle?
  //however, this trap profile runs extremely smoothly without pid. it just needs to be tuned. so this is relatively high priority
  //fixes many of our sotm problems
  private final TrapezoidProfile trapProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(700, 700*4));

  SparkFlex motor = new SparkFlex(14, MotorType.kBrushless);
  TurretSim sim = new TurretSim(motor);

  TrapezoidProfile.State goalState =  new TrapezoidProfile.State(0,0);
  TrapezoidProfile.State currentState =  new TrapezoidProfile.State(-180,0);

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.2, 0.0143);

  
  /** Creates a new Turret. */
  public Turret() {
    motor.configure(getMotorConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    CRTAbsoluteEncoder.getInstance().setParams(kTurretGearToothCount, kGear1ToothCount, kGear2ToothCount, true);
    CRTAbsoluteEncoder.getInstance().setRelativeEncoder(motor.getEncoder());
    CRTAbsoluteEncoder.getInstance().setEncoder1(motor.getAbsoluteEncoder());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("shooter/turret/applied", motor.getAppliedOutput());
    SmartDashboard.putNumber("shooter/turret/current", motor.getOutputCurrent());
    SmartDashboard.putNumber("shooter/turret/target", targetPosition.in(Degrees));
    SmartDashboard.putNumber("shooter/turret/tolerance", tolerance.in(Degrees));
    SmartDashboard.putBoolean("shooter/turret/ontarget", getOnTarget());
    SmartDashboard.putNumber("shooter/turret/CRTpos", CRTAbsoluteEncoder.getInstance().getPosition().in(Degrees));
    SmartDashboard.putNumber("shooter/turret/e1", motor.getAbsoluteEncoder().getPosition());
    SmartDashboard.putNumber("shooter/turret/relPos", motor.getEncoder().getPosition());
    SmartDashboard.putNumber("shooter/turret/velocity", motor.getEncoder().getVelocity());
  }

  /**
   * @return degrees
   */
  public Angle getAngle() {
    return Degrees.of(motor.getEncoder().getPosition());
  }
  
  public AngularVelocity getVelocity() {
    return DegreesPerSecond.of(motor.getEncoder().getVelocity());
  }

  public Command setAngle(Supplier<Angle> position, Supplier<Angle> tolerance) {
    return run(()->{
      SmartDashboard.putNumber("shooter/turret/preClampedTarget", position.get().in(Degrees));
      //ONLY WORKS ASSUMING TOTAL RANGE <360
      double normalizedPosition = position.get().in(Degrees) > kMaxRotation ? position.get().in(Degrees) - 360.0 : position.get().in(Degrees);
      this.targetPosition = Degrees.of(MathUtil.clamp(normalizedPosition, kMinRotation, kMaxRotation));
      this.tolerance = tolerance.get();
      motor.getClosedLoopController().setSetpoint(
        targetPosition.in(Degrees),
        ControlType.kPosition);
      });
  }

  public Command setVoltage(DoubleSupplier voltage) {
    return run(()->{
      motor.setVoltage(voltage.getAsDouble());
    }).finallyDo(()->motor.setVoltage(0));
  }

  public Command setAngle(Supplier<TargetingSystem.ShooterState> targetSupplier) {
    return setAngle(()->targetSupplier.get().turretAngle, ()->targetSupplier.get().turretTolerance);
  }

  public Command setAngleTrap(Supplier<Angle> position, Supplier<Angle> tolerance) {
    return startRun(()->{
        double normalizedPosition = position.get().in(Degrees) > kMaxRotation ? position.get().in(Degrees) - 360.0 : position.get().in(Degrees);
        double clampedPosition = MathUtil.clamp(
          normalizedPosition,
          kMinRotation+3, kMaxRotation-3
        );
        goalState = new TrapezoidProfile.State(clampedPosition, 0);
        currentState = new TrapezoidProfile.State(getAngle().in(Degrees), getVelocity().in(DegreesPerSecond));
        this.targetPosition = Degrees.of(normalizedPosition);
        this.tolerance = tolerance.get();
      },
      ()->{
        double normalizedPosition = position.get().in(Degrees) > kMaxRotation ? position.get().in(Degrees) - 360.0 : position.get().in(Degrees);
        double clampedPosition = MathUtil.clamp(
          normalizedPosition,
          kMinRotation+3, kMaxRotation-3
        );
        goalState = new TrapezoidProfile.State(clampedPosition, 0);
        this.targetPosition = Degrees.of(normalizedPosition);
        this.tolerance = tolerance.get();
        currentState = trapProfile.calculate(0.02, currentState, goalState);
        double ff = feedforward.calculate(currentState.velocity);
        SmartDashboard.putNumber("shooter/turret/trapposition", currentState.position);
        SmartDashboard.putNumber("shooter/turret/trapvelocity", currentState.velocity);
        motor.getClosedLoopController().setSetpoint(
          currentState.position,
          ControlType.kPosition,
          ClosedLoopSlot.kSlot0,
          ff,
          ArbFFUnits.kVoltage
        );
      });
  }

  public Command setAngleTrap(Supplier<TargetingSystem.ShooterState> targetSupplier) {
    return setAngleTrap(()->targetSupplier.get().turretAngle, ()->targetSupplier.get().turretTolerance);
  }

  public boolean getOnTarget() {
    if (targetPosition.in(Degrees) < kMaxRotation && targetPosition.in(Degrees) > kMinRotation) {
      return MathUtil.isNear(targetPosition.in(Degrees), motor.getEncoder().getPosition(), tolerance.in(Degrees));
    }
    return false;
  }

  private SparkBaseConfig getMotorConfig() {
    SparkFlexConfig config = new SparkFlexConfig();

    config
      .smartCurrentLimit(60)
      .idleMode(IdleMode.kBrake)
      // giving positive power should turn the turret CCW
      .inverted(false);

    config.encoder
      .positionConversionFactor(kGearing * 360.0)
      .velocityConversionFactor(kGearing * 360.0 / 60.0);

    config.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.3 * 2.5 * 12 / 90.0 )
      // .p(0)
      .positionWrappingEnabled(false)
      .maxOutput(outPut)
      .minOutput(-outPut);
      

    config.softLimit
      .forwardSoftLimit(kMaxRotation)
      // .forwardSoftLimit(-135)
      .forwardSoftLimitEnabled(true)
      .reverseSoftLimit(kMinRotation)
      // .reverseSoftLimit(-225)
      .reverseSoftLimitEnabled(true);

    config.absoluteEncoder
      .positionConversionFactor(360.0)
      .inverted(true);

    config.apply(new SignalsConfig()
      .appliedOutputPeriodMs(10) //frame 0 : Applied output, faults
      .primaryEncoderVelocityPeriodMs(200) //frame 1 : Velocity, temp, input voltage, stator current
      // .primaryEncoderPositionPeriodMs(20) // frame 2 : Motor position
      .absoluteEncoderPositionPeriodMs(100) //frame 5 
      // .absoluteEncoderVelocityPeriodMs(20) //frame 6
    );

      
    return config;
  }
}
