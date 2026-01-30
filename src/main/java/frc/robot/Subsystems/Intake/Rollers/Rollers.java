// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake.Rollers;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase {

  SparkFlex motor = new SparkFlex(35, MotorType.kBrushless);

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
    .p(0);

    config.closedLoop.maxMotion
    .maxAcceleration(20)
    .cruiseVelocity(6000/60);

    config
    .idleMode(IdleMode.kCoast)
    .inverted(false)
    .smartCurrentLimit(20)
    .voltageCompensation(11)
    ;

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setDefaultCommand(stop());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Rollers/DutyCycle" , motor.getAppliedOutput());
    SmartDashboard.putNumber("Rollers/OutputCurrent" , motor.getOutputCurrent());
  }

  @Override
  public void simulationPeriodic(){
    sim.update();
    SmartDashboard.putNumber("Rollers/SimVelocity" , sim.getVelocity().in(RPM));
  }


  public Command setVelocity(double velocity){
  return run(()->{
      motor
      .getClosedLoopController()
      .setSetpoint(velocity, ControlType.kMAXMotionVelocityControl);
    });
  }

  public Command intake(){
    return setVelocity(5);
  }


 public Command eject(){
    return setVelocity(-5);
  }

  public Command stop(){
    return run(()->{
        motor.stopMotor();
    });
  }


}
