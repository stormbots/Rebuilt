// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Spindexer.UpGoer;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
// import edu.wpi.first.units.Units.Degree;
/** Add your docs here. */
public class UpGoerSim {
    SparkFlex UpGoerMotor;
    SparkFlexSim SimUpGoer;

    //final Angle startingAngle = Degree.of(40);

    public UpGoerSim(SparkFlex UpGoerMotor){
        this.UpGoerMotor = UpGoerMotor;
        this.SimUpGoer = new SparkFlexSim(UpGoerMotor, DCMotor.getNeoVortex(1));
        
    }

    FlywheelSim UpGoerFlywheelSim = new FlywheelSim(
    LinearSystemId.createFlywheelSystem(
      DCMotor.getNeoVortex(1),
      0.002,
      1
    ),
    DCMotor.getNeoVortex(1)
  );

  public void update(){
    var dt = 0.02;
    var vbus = 12;
    UpGoerFlywheelSim.setInputVoltage(SimUpGoer.getAppliedOutput() * vbus);
    UpGoerFlywheelSim.update(dt);
    SimUpGoer.iterate(UpGoerFlywheelSim.getAngularVelocityRPM(), vbus, dt);
  }

  public AngularVelocity getUpGoerSpeed(){
    return UpGoerFlywheelSim.getAngularVelocity();
  }

}
