// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake.IntakeExtension;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class IntakeExtension extends SubsystemBase {
  SparkFlex motor = new SparkFlex(9, MotorType.kBrushless);
  SparkFlex followerMotor = new SparkFlex(10, MotorType.kBrushless);

  IntakeExtensionSim sim = new IntakeExtensionSim(motor);

  /** Creates a new IntakeExtension. */
  public IntakeExtension() {
    var config = new  SparkFlexConfig();
    double factor = 1;
    config.encoder
    .positionConversionFactor(factor)
    .velocityConversionFactor(factor / 60);

    var absfactor = 360;
    config.absoluteEncoder
    .inverted(false)
    .positionConversionFactor(absfactor)
    .velocityConversionFactor(absfactor / 60);

    config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    config.closedLoop.feedForward
    .svacr(0, 0, 0, 0, 0);
    
    config.closedLoop.p(8/90.0);

    config.closedLoop.maxMotion
    .maxAcceleration(360/2*4)
    .cruiseVelocity(360/2);

    config
    .idleMode(IdleMode.kCoast)
    .inverted(false)
    .smartCurrentLimit(20)
    .voltageCompensation(11);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //Apply the follower configuration, and re-use any applicable configs for the other side
    config.follow(motor,true);
    followerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    new Trigger(DriverStation::isEnabled)
    .onTrue(setIdleMode(IdleMode.kCoast))
    .onFalse(setIdleMode(IdleMode.kBrake));
    
    // setDefaultCommand(up());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    motor.getAppliedOutput();
    motor.getOutputCurrent();
    motor.getAbsoluteEncoder().getPosition();
    SmartDashboard.putNumber("Intake/Extension/OutputCurrent", motor.getOutputCurrent());
    SmartDashboard.putNumber("Intake/Extension/Dutycycle", motor.getAppliedOutput());
    SmartDashboard.putNumber("Intake/Extension/enc angle", motor.getAbsoluteEncoder().getPosition());
  }

  @Override
  public void simulationPeriodic(){
    sim.update();
    SmartDashboard.putNumber("Intake/Extension/SimAngle", sim.getAngle().in(Degree));
  }

  public Command setAngle(double degrees){
    return run(()->{
      motor
      .getClosedLoopController()
      //FIXME: Get kSmartMaxMotion working. Weird issues in sim.
      .setSetpoint(degrees, ControlType.kPosition);
    });
  }


  public Command up(){
    return setAngle(90);
  }

  public Command down(){
    return setAngle(0);
  }

  public Angle getAngle(){
    return Degrees.of(motor.getAbsoluteEncoder().getPosition());
  };


  private Command setIdleMode(IdleMode mode){
    return Commands.runOnce(()->{
      var config = new SparkFlexConfig().idleMode(mode);
      motor.configure(
        config,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
    });
  }
}
