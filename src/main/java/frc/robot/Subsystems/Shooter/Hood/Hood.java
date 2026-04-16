// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter.Hood;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.TargetingSystem.TargetingSystem;

public class Hood extends SubsystemBase {

  public static final double kGearing = (1.0 / 15.0) * (10.0 / 164.0);

  public static final int kPreHomeCurrentLimit = 10;
  //CANNOT be higher than 20, ITS A NEO 550
  public static final int kPostHomeCurrentLimit = 20;
  public static final double kHomeCurrentThreshold = 4.0;

  public static final double homeAngle = 0.0;
  //minimum reachable should be slightly higher than hard limit
  public static final double minAngle = homeAngle+0.5;
  public static final double maxAngle = 43.0;

  private boolean homed = true; //TODO:make actual homed thingy
  private boolean isSuppressed = false;

  private Angle targetAngle = Degrees.of(minAngle);
  private Angle tolerance = Degrees.of(3); 



  SparkMax motor = new SparkMax(15, MotorType.kBrushless);
  HoodSim sim = new HoodSim(motor);


  Trigger isAtHome = new Trigger(()->!homed && motor.getOutputCurrent() > kHomeCurrentThreshold).debounce(0.1);

  /** Creates a new Hood. */
  public Hood() {
    motor.configure(getMotorConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setDefaultCommand(setAngle(()->Degrees.of(0.0), ()->Degrees.of(0.5)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("shooter/hood/current", motor.getOutputCurrent());
    SmartDashboard.putNumber("shooter/hood/voltage", motor.getAppliedOutput()*motor.getBusVoltage());
    SmartDashboard.putNumber("shooter/hood/position", motor.getEncoder().getPosition());
    SmartDashboard.putBoolean("shooter/hood/homed", homed);
    SmartDashboard.putBoolean("shooter/hood/suppressed", isSuppressed);
  }

  @Override
  public void simulationPeriodic(){
    sim.update();
  }

  private SparkBaseConfig getMotorConfig(){
    SparkBaseConfig config = new SparkMaxConfig();

    config.
      smartCurrentLimit(kPreHomeCurrentLimit)
      .idleMode(IdleMode.kBrake)
      .inverted(true)
    ;

    
    config.encoder
      .positionConversionFactor(kGearing * 360.0)
      .velocityConversionFactor(kGearing * 360.0 / 60.0)
    ;

    config.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(2 * 0.3 / 30 * 5 * 1.2)
    ;

    //do not enable soft limits until homed
    config.softLimit
      .forwardSoftLimit(maxAngle)
      .forwardSoftLimitEnabled(false)
      .reverseSoftLimit(minAngle)
      .reverseSoftLimitEnabled(false)
    ;

    config.apply(new SignalsConfig()
      .appliedOutputPeriodMs(30) //frame 0 : Applied output, faults
      .primaryEncoderVelocityPeriodMs(200) //frame 1 : Velocity, temp, input voltage, stator current
      // .primaryEncoderPositionPeriodMs(20) // frame 2 : Motor position
      // .absoluteEncoderPositionPeriodMs(20) //frame 5 
      // .absoluteEncoderVelocityPeriodMs(20) //frame 6
    );
    
    return config;
  }

  public Angle getAngle(){
    return Degrees.of(motor.getEncoder().getPosition());
  }

  public Command setAngle(Supplier<Angle> angle, Supplier<Angle> tolerance){
    return run(()->{
      this.targetAngle = angle.get();
      var targetDegrees = isSuppressed ? 0 : this.targetAngle.in(Degrees);
      this.tolerance = tolerance.get();
      motor.getClosedLoopController().setSetpoint(
        targetDegrees,
        ControlType.kPosition
      );
    });
  }

  public void stop(){
    motor.stopMotor();
  }

  public Command setAngle(Supplier<TargetingSystem.ShooterState> targetSupplier){
    return setAngle(()->targetSupplier.get().hoodAngle, ()->targetSupplier.get().hoodTolerance);
  }

  public boolean getOnTarget(){
    if(isSuppressed) return false;
    return MathUtil.isNear(targetAngle.in(Degrees), motor.getEncoder().getPosition(), tolerance.in(Degrees) + 2.5);
  }

  private void setHomeableConfig(){
    SparkBaseConfig config = new SparkMaxConfig();
    
    config.smartCurrentLimit(kPreHomeCurrentLimit);

    config.softLimit
      .forwardSoftLimitEnabled(false)
      .reverseSoftLimitEnabled(false)
    ;

    motor.configureAsync(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  //TODO: move this out into shooter, this itself shouldn't count as a subsystem.
  public Command homingCommand(){
    return new FunctionalCommand(
      ()->{
        homed = false;
        setHomeableConfig();
      }, 
      ()->motor.set(-0.1), 
      (interrupted)->{
        if(!interrupted){
          setHomed();
        }
      }, 
      isAtHome, 
      this
    )
    .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
    .withTimeout(5)
    ;
  }

  private void setHomed(){
    SparkBaseConfig config = new SparkMaxConfig();

    config.smartCurrentLimit(20);
    
    config.softLimit
      .forwardSoftLimitEnabled(true)
      .reverseSoftLimitEnabled(true)
    ;

    motor.getEncoder().setPosition(homeAngle);

    motor.configureAsync(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    homed=true;
  }

  public Command stow(){
    return setAngle(()->Degrees.of(0), ()->Degrees.of(10));
  }

  /** Bring the hood down temporarily without disrupting existing commands */
  public Command suppressForTrench(){
    return Commands.startEnd(()->isSuppressed=true, ()->isSuppressed=false);
  }

  /** Indiate if something is suppressing the hood */
  public boolean isSuppressed(){
    return isSuppressed;
  }
}
