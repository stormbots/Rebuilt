// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter.Hood;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
// import edu.wpi.first.units.Units.Degree;
/** Add your docs here. */
public class HoodSim {
    SparkMax motor;
    SparkMaxSim sim;

    //final Angle startingAngle = Degree.of(40);

    public HoodSim(SparkMax hoodMotor){
      this.motor = hoodMotor;
      this.sim = new SparkMaxSim(hoodMotor, DCMotor.getNeo550(1));
        
    }

    FlywheelSim plant = new FlywheelSim(
    LinearSystemId.createFlywheelSystem(
      DCMotor.getNeoVortex(1),
      0.002,
      20
    ),
    DCMotor.getNeo550(1).withReduction(20)
  );

  public void update(){
    var dt = 0.02;
    var vbus = 12;
    plant.setInputVoltage(sim.getAppliedOutput() * vbus);
    plant.update(dt);
    sim.iterate(plant.getAngularVelocityRPM(), vbus, dt);
  }

  public AngularVelocity getVelocity(){
    return plant.getAngularVelocity();
  }

}
