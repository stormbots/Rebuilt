// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter.Turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** Add your docs here. */
public class TurretSim {
    SparkFlex shooterMotor;
    SparkFlexSim simShooterMotor;

    public TurretSim(SparkFlex turretMotor){
        this.shooterMotor = turretMotor;
        simShooterMotor = new SparkFlexSim(turretMotor, DCMotor.getNeoVortex(1));
        // plant.setState(shooterMotor.getEncoder().getPosition(), 0);
        // simShooterMotor.setPosition(Turret.kRotationToRobotForward.in(Degree));
    }

  Angle minAngle = Degrees.of(-Turret.kMaxRotation);
  Angle maxAngle = Degrees.of(Turret.kMaxRotation);

  FlywheelSim plant = new FlywheelSim(
    LinearSystemId.createFlywheelSystem(
      DCMotor.getNeoVortex(1), 0.002016125, 12
    ),
    DCMotor.getNeoVortex(1)
  );
  
  public void update(){
    var dt = 0.02;
    var vbus = 12;

    plant.setInputVoltage(simShooterMotor.getAppliedOutput()*vbus);
    plant.update(dt);
    simShooterMotor.iterate(
      DegreesPerSecond.of(plant.getAngularVelocityRPM()).in(DegreesPerSecond),
      vbus, dt
    );
  }

}
