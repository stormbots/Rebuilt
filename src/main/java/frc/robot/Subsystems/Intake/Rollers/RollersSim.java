// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake.Rollers;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** Add your docs here. */
public class RollersSim {
  SparkFlex motor;
  SparkFlexSim simMotor;

  public RollersSim(SparkFlex shooterMotor){
    this.motor = shooterMotor;
    simMotor = new SparkFlexSim(shooterMotor, DCMotor.getNeoVortex(1));
  }

  FlywheelSim plant = new FlywheelSim(
    LinearSystemId.createFlywheelSystem(
      DCMotor.getNeoVortex(1), 0.00002016125*3, 1
    ),
    DCMotor.getNeoVortex(1)
  );

  public void update() {
    var dt = 0.02;
    var vbus = 12;

    plant.setInputVoltage(simMotor.getAppliedOutput()*vbus);
    plant.update(dt);
    simMotor.iterate(
      plant.getAngularVelocity().in(RPM),
      vbus, dt
    );
  }

  /** Access the state of the simulated plant */
  public AngularVelocity getVelocity(){
    return plant.getAngularVelocity();
  }
}
