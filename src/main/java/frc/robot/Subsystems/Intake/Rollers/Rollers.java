// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake.Rollers;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase {

  SparkFlex motor = new SparkFlex(11, MotorType.kBrushless);

  RollersSim sim = new RollersSim(motor);

  /** Creates a new Rollers. */
  public Rollers() {
    //configure the motor
    var config = new  SparkFlexConfig();
    double factor = 1; //convert to surface speed of roller
    config.encoder
    .positionConversionFactor(factor)
    .velocityConversionFactor(factor/60);

    config.closedLoop.feedForward
    .sva(0, 12/6000.0/factor, 0);

    config.closedLoop
    .p(1/1000.0);

    config.closedLoop.maxMotion
    .maxAcceleration(20)
    .cruiseVelocity(6000/60)
    .allowedProfileError(50)
    ;

    config
    .idleMode(IdleMode.kCoast)
    .inverted(true)
    .smartCurrentLimit(45)
    .voltageCompensation(11);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setDefaultCommand(stop());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/Rollers/DutyCycle", motor.getAppliedOutput());
    SmartDashboard.putNumber("Intake/Rollers/Current", motor.getOutputCurrent());
    // SmartDashboard.putNumber("Intake/Rollers/Current", motor.getAppliedOutput());

    // SmartDashboard.putNumber("Intake/Rollers/Position", getPosition().in(Units.Degrees));
    // SmartDashboard.putNumber("Intake/Rollers/Velocity", getVelocity().in(Units.DegreesPerSecond));
    SmartDashboard.putString("Intake/Rollers/Command", getCurrentCommand()==null ? "None" : getCurrentCommand().getName() );
  }

  @Override
  public void simulationPeriodic(){
    sim.update();
    SmartDashboard.putNumber("Intake/Rollers/SimVelocity" , sim.getVelocity().in(RPM));
  }


  public Command setVelocity(double rpm){
    return run(()->{
      motor
      .getClosedLoopController()
      .setSetpoint(rpm, ControlType.kVelocity);
    });
  }

  public Command intake(){
    // return setVelocity(100);
    return setVoltage(8.0);
  }

 public Command eject(){
    // return setVelocity(-100);
    return setVoltage(-6);
  }

  public Command stop(){
    return Commands.sequence(
          run(motor::stopMotor)
    );
  }

  public AngularVelocity getVelocity(){
    return RPM.of(motor.getEncoder().getVelocity());
  }

  public Angle getPosition(){
    return Rotations.of(motor.getEncoder().getPosition());
  }

  public Command setVoltage(double volts){
    return run(()->{
      motor.setVoltage(volts);
    });
  }
  
}
