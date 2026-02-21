// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter.Hood;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Hood extends SubsystemBase {

  public static final double kGearing = (1.0 / 15.0) * ( 10.0 / 164.0 );

  public static final int kPreHomeCurrentLimit = 10;
  //CANNOT be higher than 20, ITS A NEO 550
  public static final int kPostHomeCurrentLimit = 20;
  public static final double kHomeCurrentThreshold = 8.0;

  public static final double homeAngle = 0.0;
  //minimum reachable should be slightly higher than hard limit
  public static final double minAngle = homeAngle+0.5;
  public static final double maxAngle = 0.0;

  private boolean homed = false;

  private Angle targetAngle = Degrees.of(minAngle);
  private Angle tolerance = Degrees.of(3); 

  SparkMax motor = new SparkMax(15, MotorType.kBrushless);

  Trigger isAtHome = new Trigger(()-> !homed && motor.getOutputCurrent()>kHomeCurrentThreshold );

  /** Creates a new Hood. */
  public Hood() {
    motor.configure(getMotorConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
      .p(0.0)
    ;

    //do not enable soft limits until homed
    config.softLimit
      .forwardSoftLimit(maxAngle)
      .forwardSoftLimitEnabled(false)
      .reverseSoftLimit(minAngle)
      .reverseSoftLimitEnabled(false)
    ;

    return config;
  }

  public Angle getAngle(){
    return Degrees.of(motor.getEncoder().getPosition());
  }

  private void setAngle(Angle angle, Angle tolerance){
    if(homed){
      this.targetAngle = angle;
      this.tolerance = tolerance;
      motor.getClosedLoopController().setSetpoint(
        angle.in(Degrees),
        ControlType.kPosition
      );
    }
  }

  public Command setAngleCommand(Angle angle, Angle tolerance){
    return run(()->setAngle(angle, tolerance));
  }

  public boolean getOnTarget(){
    return homed && MathUtil.isNear(targetAngle.in(Degrees), motor.getEncoder().getPosition(), tolerance.in(Degrees));
  }


  private void setCurrentLimit(int limit){
    SparkBaseConfig config = new SparkMaxConfig();
    
    config.smartCurrentLimit(limit);

    motor.configureAsync(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  //TODO: move this out into shooter, this itself shouldn't count as a subsystem.
  public Command homingCommand(){
    return new FunctionalCommand(
      ()->{
        homed=false;
        setCurrentLimit(kPreHomeCurrentLimit);
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

    motor.configureAsync(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    homed=true;
  }
}
