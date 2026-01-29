// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake.IntakeExtension;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public class IntakeExtensionSim {
    SparkFlex armMotor;
    SparkFlexSim simArmMotor;

    final Angle startingAngle=Degree.of(40);

    public IntakeExtensionSim(SparkFlex armMotor){
        this.armMotor = armMotor;
        simArmMotor = new SparkFlexSim(armMotor, DCMotor.getNeoVortex(1));
        simArmMotor.setPosition(startingAngle.in(Degree));
        simArmMotor.getRelativeEncoderSim().setPosition(startingAngle.in(Degree));
        simArmMotor.getAbsoluteEncoderSim().setPosition(startingAngle.in(Degree));
    }
  
  SingleJointedArmSim simArm = new SingleJointedArmSim(
    DCMotor.getNeoVortex(1), 
    20*90/30.362,
    0.2,
    0.5,
    Degrees.of(-40).in(Radians),
    Degrees.of(100).in(Radians),
    false,
    startingAngle.in(Radians)
  );

  public void update() {
    var dt = 0.02;
    var vbus = 12;

    simArm.setInputVoltage(simArmMotor.getAppliedOutput()*vbus);
    simArm.update(dt);

    double velocity = RadiansPerSecond.of(simArm.getVelocityRadPerSec()).in(DegreesPerSecond);
    simArmMotor.getAbsoluteEncoderSim().iterate(velocity, dt);
    simArmMotor.iterate(
      RadiansPerSecond.of(simArm.getVelocityRadPerSec()).in(DegreesPerSecond),
      vbus, dt
    );
  }


  /** Access the state of the simulated plant */
  public Angle getAngle(){
    return Radians.of(simArm.getAngleRads());
  }

}

