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
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase {

  SparkFlex motor = new SparkFlex(11, MotorType.kBrushless);
  SparkFlex follower = new SparkFlex(20, MotorType.kBrushless);

  RollersSim sim = new RollersSim(motor);

  /** Creates a new Rollers. */
  public Rollers() {
    //configure the motor
    var config = new  SparkFlexConfig();
    double factor = 1; //convert to surface speed of roller
    config.encoder
      .positionConversionFactor(factor)
      .velocityConversionFactor(factor);

    config.closedLoop.feedForward
      // .sva(0, 12/6000.0/factor, 0);
      .sva(0, 0, 0);

    config.closedLoop
      // .p(1/350.0*0.5*0.5*1.2);
      .p(0);

    // config.closedLoop.maxMotion
    //   .maxAcceleration(20)
    //   .cruiseVelocity(6000/60)
    //   .allowedProfileError(50)
    //   ;

    config
      .idleMode(IdleMode.kCoast)
      .inverted(false)
      .smartCurrentLimit(30)
      // .voltageCompensation(11)
    ;

    config.apply(new SignalsConfig()
    .appliedOutputPeriodMs(5) //frame 0 // faster for improved follower perf
    .primaryEncoderVelocityPeriodMs(200) //frame 1
    .primaryEncoderPositionPeriodMs(30) // frame 2
    // .absoluteEncoderPositionPeriodMs(20) //frame 5
    // .absoluteEncoderVelocityPeriodMs(20) //frame 6
    );

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    
    //Configure the follower with additional specific details
    config
      .follow(motor,true)
      ;
  
    config.apply(new SignalsConfig()
    .appliedOutputPeriodMs(100) //frame 0
    // .primaryEncoderVelocityPeriodMs(20) //frame 1
    // .primaryEncoderPositionPeriodMs(20) // frame 2
    // .absoluteEncoderPositionPeriodMs(20) //frame 5
    // .absoluteEncoderVelocityPeriodMs(20) //frame 6
    );

    follower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    setDefaultCommand(stop());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/Rollers/voltage", motor.getBusVoltage()*motor.getAppliedOutput());
    SmartDashboard.putNumber("Intake/Rollers/DutyCycle", motor.getAppliedOutput());
    SmartDashboard.putNumber("Intake/Rollers/Current", motor.getOutputCurrent());
    // SmartDashboard.putNumber("Intake/Rollers/Follower/Current", follower.getOutputCurrent());
    SmartDashboard.putNumber("Intake/Rollers/Velocity", getVelocity().in(Units.RPM));
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
    // return setVelocity(8.0/12.0 *6000.0);
    return setVoltage(8);
  }

 public Command eject(){
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
