// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter.Turret;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.stormbots.CRTAbsoluteEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

  public static final double TURRET_GEAR_TOOTH_COUNT = 100;
  public static final double GEAR_1_TOOTH_COUNT = 16;
  public static final double GEAR_2_TOOTH_COUNT = 17;
  private static final CRTAbsoluteEncoder crtAbsoluteEncoder = new CRTAbsoluteEncoder(TURRET_GEAR_TOOTH_COUNT, GEAR_1_TOOTH_COUNT, GEAR_2_TOOTH_COUNT, true);
  
  public static final double GEARING = 1.0;

  public static final double MAXROTATION = 540.0 / 2.0;


  SparkFlex motor = new SparkFlex(99, MotorType.kBrushless);

  /** Creates a new Turret. */
  public Turret() {

    motor.configure(getMotorConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    motor.getEncoder().setPosition(getTurretAngleAbsolute());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * @return degrees
   */
  public double getAngle(){
    return motor.getEncoder().getPosition();
  }

  /**
   * @param angle degrees
   */
  private void setAngle(double angle){
  }

  private void setAngle(Rotation2d angle) {
    setAngle(angle.getDegrees());
  }

  public double getTurretAngleAbsolute(){
    //Since encoders cannot be accessed til runtime, will likely be passed in through constructor
    //Make getAngle() take encoder positions as arguments
    //Could have better design but ykw it works for now
    double turretAngle = crtAbsoluteEncoder.getAngle(
      //replace with correct absolute encoders
      motor.getAbsoluteEncoder().getPosition(), 
      motor.getAbsoluteEncoder().getPosition()
    );

    return turretAngle;
  }

  private SparkBaseConfig getMotorConfig(){
    SparkFlexConfig config = new SparkFlexConfig();

    config
      .smartCurrentLimit(40)
      .idleMode(IdleMode.kBrake)
      //giving positive power should turn the turret CCW
      .inverted(false)
    ;

    config.encoder
      .positionConversionFactor(GEARING)
      .velocityConversionFactor(GEARING / 60.0)
      //Turning the turret CCW should increase position
      .inverted(false)
    ;

    config.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.0)
    ;

    config.softLimit
      .forwardSoftLimit(MAXROTATION)
      .forwardSoftLimitEnabled(true)
      .reverseSoftLimit(-MAXROTATION)
      .reverseSoftLimitEnabled(true)
    ;

    return config;
  }
}
