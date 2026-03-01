// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Climber.Grabber;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Grabber extends SubsystemBase {
  SparkFlex motor = new SparkFlex(30, MotorType.kBrushless);;
  private boolean isHomed = false;
  private String name;
  private final int kHomeCurrentThreshold = 5;
  private final int kHomeCurrentMaxOutput = 8;
  private final int kGrabberCurrentMax = 10;

  Trigger isHomingCurrentReached = new Trigger(()->{
    return Math.abs(motor.getOutputCurrent()) <= kHomeCurrentThreshold;
  }).debounce(0.5)
  ;

  public Grabber() {

    //single motor
    //calibrate encoder to correct angle
    //can tell if it's retracted
    //maybe: Use as data to tell if we're successfully grabbled and can climb
    //PID: Maybe? Probably
    var config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake);
    config.inverted(false);
    config.smartCurrentLimit(kGrabberCurrentMax);
    config.openLoopRampRate(0.05);
    //TODO Configure the encoder conversion
    var conversionfactor=1/(6/57.71); //1 divided by whatever number you determined
    conversionfactor = 1; 
		config.encoder
    .positionConversionFactor(1/conversionfactor)
    .velocityConversionFactor(1/conversionfactor/60.0)
    ;

    config.closedLoop
    .p(12/2.0)
    ;

    config.softLimit
    .forwardSoftLimit(90)
    .reverseSoftLimit(0.0)
    .forwardSoftLimitEnabled(true)
    .reverseSoftLimitEnabled(true)
    ;

    motor.configure(
      config,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    // new Trigger(DriverStation::isEnabled)
		// .and(()->isHomed==false)
		// .whileTrue(goHome());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // private Command setPosition(...)
  private Command setPosition(Angle angle){
    var degrees = angle.in(Degree);
    return run(()->{
        motor
        .getClosedLoopController()
        .setSetpoint(degrees, ControlType.kPosition);
    });
  }

  private Angle getPosition(){
    return Degrees.of(motor.getEncoder().getPosition());
  }

  public Command grab(){
    return setPosition(Degrees.of(90));
  }

  public Command retract(){
    return setPosition(Degrees.of(0));
  }

  public Command goHome(){
    return new FunctionalCommand(
      ()->{
        isHomed=false;
        enableBottomLimit(false);
        setCurrentLimit(kHomeCurrentMaxOutput);
      },
      ()->{motor.setVoltage(-12);},
      (cancelled)->{
        if(cancelled==false){
          isHomed = true;
          enableBottomLimit(true);
          setCurrentLimit(kGrabberCurrentMax);
          motor.getEncoder().setPosition(0);
        }
        else{}
        motor.stopMotor();
      }, 
      isHomingCurrentReached, 
      this
    )
    .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
		.withTimeout(Seconds.of(5))
    .withName(name+"goHome")
    ;
  }

  private void setCurrentLimit(int amps){
    var config = new SparkFlexConfig();
    config.smartCurrentLimit(amps);

    motor.configureAsync(
      config,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
    );
  }

  private void enableBottomLimit(boolean enabled){
    var config = new SparkFlexConfig();
    config.softLimit.reverseSoftLimitEnabled(enabled);

    motor.configureAsync(
      config,
      ResetMode.kNoResetSafeParameters,
      PersistMode.kNoPersistParameters
    );
  }

  public Trigger isPossiblyConnected = new Trigger(() -> {
    return getPosition().isNear(Degrees.of(90), Degrees.of(10));    
  });

  public Trigger isRetracted = new Trigger(() -> {
    return getPosition().isNear(Degrees.of(0), Degrees.of(10));    
  });
  

}
