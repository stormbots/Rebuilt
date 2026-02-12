// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Climber.ClimberExtension;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ClimberExtension extends SubsystemBase {

  SparkFlex motor; //handled in constructor
  public ClimberExtensionSim sim; //handled in constructor

  private boolean isHomed = false;
  private final int kHomeCurrentThreshold = 4;
  private final int kClimbingCurrentThreshold = 20;

  private  String name="";

  Trigger isHomingCurrentReached = new Trigger(()->{
    return motor.getOutputCurrent() <= kHomeCurrentThreshold;
  }).debounce(0.1)
  ;

  /** Creates a new ClimberRight. */
  public ClimberExtension(
    String name,
    int motorID,
    boolean inverted,
    Distance movementRange
  ) {
    motor = new SparkFlex(motorID, MotorType.kBrushless);
    sim = new ClimberExtensionSim(motor);
    this.name = name;

    var config = new SparkFlexConfig();
    config.idleMode(IdleMode.kCoast);
    config.inverted(inverted);
    config.smartCurrentLimit(4);
    config.openLoopRampRate(0.05);

    //TODO Configure the encoder conversion
    var conversionfactor=6/57.71; //1 divided by whatever number you determined
		config.encoder
    .positionConversionFactor(1/conversionfactor)
    .velocityConversionFactor(1/conversionfactor/60.0)
    ;

    // config.closedLoop
    // .p(12/6)
    // ;

    config.softLimit
    .forwardSoftLimit(movementRange.in(Inches))
    .reverseSoftLimit(0)
    .forwardSoftLimitEnabled(true)
    .reverseSoftLimitEnabled(false)
    ;

    motor.configure(
      config,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    new Trigger(DriverStation::isEnabled)
		.and(()->isHomed==false)
		.whileTrue(goHome());
  }

  public Command goHome(){
    return new FunctionalCommand(
      ()->{
        isHomed=false;
        enableBottomLimit(false);
        setCurrentLimit(kHomeCurrentThreshold);
      },
      ()->{motor.set(-0.5);}, 
      (cancelled)->{
        if(cancelled==false){
          isHomed = true;
          enableBottomLimit(true);
          setCurrentLimit(kClimbingCurrentThreshold);
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
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber/"+name+"/height", motor.getEncoder().getPosition());
    SmartDashboard.putNumber("Climber/"+name+"/velocity", motor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Climber/"+name+"/current", motor.getOutputCurrent());
    SmartDashboard.putNumber("Climber/"+name+"/output", motor.getAppliedOutput());
    SmartDashboard.putBoolean("Climber/"+name+"/homed", isHomed);
  }

  @Override
  public void simulationPeriodic(){
    sim.update();
  }

  public Command setHeight(Distance height){
    var inches = height.in(Inches);
    return run(()->{
        motor
        .getClosedLoopController()
        .setSetpoint(inches, ControlType.kPosition);
    });
  }

  public Distance getHeight(){
    return Inches.of(motor.getEncoder().getPosition());
  }

  public boolean isHomed(){
    return isHomed;
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

  public Command setOutput(double output){
    return run(() -> {
      motor.set(output);
    })
    .finallyDo(motor::stopMotor)
    ;
  }


}
