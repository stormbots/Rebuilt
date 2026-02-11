// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Climber.ClimberTrolley;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;


public class ClimberExtensionSim {
  SparkFlex elevatorMotor;
  SparkFlexSim simElevatorMotor;
  
  public ClimberExtensionSim(
    SparkFlex elevatorMotor){
    this.elevatorMotor = elevatorMotor;
    simElevatorMotor = new SparkFlexSim(this.elevatorMotor, DCMotor.getNeoVortex(1));
  }
  

  ElevatorSim simElevator = new ElevatorSim(
    DCMotor.getNeoVortex(1),
    45, 
    1, 
    Inches.of(0.8).in(Meter), 
    Inches.of(0).in(Meter), 
    Inches.of(30).in(Meter), 
    false, 
    0
  );
  

  public void update() {
    // TODO Auto-generated method stub
    var vbus = 12;
    var dt = 0.02;

    simElevator.setInputVoltage(simElevatorMotor.getAppliedOutput()*vbus);
    simElevator.update(dt);

    simElevatorMotor.iterate(
      MetersPerSecond.of(simElevator.getVelocityMetersPerSecond()).in(InchesPerSecond),
      vbus,
      dt
    );

  }

  public Distance getAngle(){
    return Meters.of(simElevator.getPositionMeters());
  }
}
