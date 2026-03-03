// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake.IntakeExtension;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
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
  SparkFlex motor;
  
  // Breaks on real robot? It shouldn't....
  // IntakeExtensionSim sim = new IntakeExtensionSim(motor);

  /** Creates a new IntakeExtension. */
  public IntakeExtension(int motorID, Boolean inverted) {
    this.motor = new SparkFlex(motorID, MotorType.kBrushless);
    var config = new  SparkFlexConfig();
    double factor = 1/9.0 * 1/5.0 * 12*36 / 360.0 ;
    //90 = all the way
    //0  = resting on top of fuel on the ground
    factor = 90/4.666661;
    // factor = 1;
    config.encoder
    .positionConversionFactor(factor)
    .velocityConversionFactor(factor / 60.0)
    ;

    //Not used in Intake system, but the port on one side is borrowed for turret
    config.absoluteEncoder
      .positionConversionFactor(360.0)
      .inverted(false)
    ;

    //TODO Set feed-forwards for intake arm?
    // config.closedLoop.feedForward
    // .svacr(0, 0, 0, 0, 0);
    
    config.closedLoop
    .p(9 / 45.0)
    ;

    config.closedLoop.maxMotion
    .maxAcceleration(360/2*4)
    .cruiseVelocity(360/2)
    .allowedProfileError(10)
    ;

    config
    .idleMode(IdleMode.kCoast)
    .inverted(inverted)
    .smartCurrentLimit(5)
    .voltageCompensation(11)
    ;

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //Apply the follower configuration, and re-use any applicable configs for the other side
    // config.follow(motor,true);

    new Trigger(DriverStation::isEnabled)
    .onTrue(setIdleMode(IdleMode.kCoast))
    .onFalse(setIdleMode(IdleMode.kBrake));
    
    //Assume the proper startup position
    motor.getEncoder().setPosition(90);
    
    setDefaultCommand(up());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Intake/Extension/getAngle", getAngle().in(Degree));
    SmartDashboard.putString("Intake/Extension/Command", getCurrentCommand()==null ? "None" : getCurrentCommand().getName() );
  }

  @Override
  public void simulationPeriodic(){
    // sim.update();
    // SmartDashboard.putNumber("Intake/Extension/SimAngle", sim.getAngle().in(Degree));
  }

  public Angle getAngle(){
    return Degrees.of(motor.getEncoder().getPosition());
  };

  public SparkAbsoluteEncoder getAbsoluteEncoder(){
    //NOTE: The absolute encoder port is borrowed for turret use
    return motor.getAbsoluteEncoder();
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

  private Command setAngle(double degrees, double arbitraryFFVolts){
    return run(()->{
      motor
      .getClosedLoopController()
      .setSetpoint(
        degrees, 
        ControlType.kPosition, 
        ClosedLoopSlot.kSlot0, 
        arbitraryFFVolts, 
        ArbFFUnits.kVoltage
      );
      //FIXME: Get kSmartMaxMotion working. Weird issues in sim.
    });
  }

  public Command up(){
    return Commands.sequence(
      setAngle(90, 0).until(()->getAngle().in(Degree) > 80),
      setAngle(90, 0)
    )
    .withName("Up")
    ;
  }

  public Command down(){
    double downTransitionAngle = 45;
    return Commands.repeatingSequence(
      run(()->motor.setVoltage(-9)).until(()->getAngle().in(Degree)<=downTransitionAngle),
      run(()->motor.stopMotor()).until(()->getAngle().in(Degree)>downTransitionAngle)
    )
    .withName("Down")
    ;
  }

  private Command setVoltage(double volts){
    return run(()->motor.setVoltage(volts));
  }
}
