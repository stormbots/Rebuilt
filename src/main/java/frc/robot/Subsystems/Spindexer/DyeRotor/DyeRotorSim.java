// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Spindexer.DyeRotor;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
// import edu.wpi.first.units.Units.Degree;
/** Add your docs here. */
public class DyeRotorSim {
    SparkFlex DyeRotorMotor;
    SparkFlexSim SimDyeRotor;

    //final Angle startingAngle = Degree.of(40);

    public DyeRotorSim(SparkFlex DyeRotorMotor){
      this.DyeRotorMotor = DyeRotorMotor;
      this.SimDyeRotor = new SparkFlexSim(DyeRotorMotor, DCMotor.getNeoVortex(1));
        
    }

    FlywheelSim DyeRotorFlywheelSim = new FlywheelSim(
    LinearSystemId.createFlywheelSystem(
      DCMotor.getNeoVortex(1).withReduction(100),
      0.2,
      1
    ),
    DCMotor.getNeoVortex(1)
  );

  public void update(){
    var dt = 0.02;
    var vbus = 12;
    DyeRotorFlywheelSim.setInputVoltage(SimDyeRotor.getAppliedOutput() * vbus);
    DyeRotorFlywheelSim.update(dt);
    SimDyeRotor.iterate(DyeRotorFlywheelSim.getAngularVelocityRPM(), vbus, dt);
  }

  public AngularVelocity getDyeRotorSpeed(){
    return DyeRotorFlywheelSim.getAngularVelocity();
  }

}
