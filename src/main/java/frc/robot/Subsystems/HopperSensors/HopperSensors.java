// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.HopperSensors;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;

public class HopperSensors extends SubsystemBase {
  private static HopperSensors instance; 

  public final int kLowCapacity = 4;
  public final int kMaxCapacity = 25;
  public int fuelInHopper = 8; // Initial fuel provided during autos

  /** Most fuel capacity is spent, and we should consider doing something else */
  public Trigger isLow = new Trigger(()->fuelInHopper<=kLowCapacity);
  /** At maximum capacity, and should not attempt to pull in more */
  public Trigger isFull = new Trigger(()->fuelInHopper>=kMaxCapacity);
  /** Fuel has room to continue intaking */
  public Trigger isNotFull = new Trigger(()->fuelInHopper<kMaxCapacity-4);

  /** Creates a new HopperSensors. */
  private HopperSensors(){
    setupRealTriggers();
    setupSimulationTriggers();
  }

  /** Return the HopperSensors singleton */
  public static HopperSensors getInstance(){
    if(instance==null)instance = new HopperSensors();
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("hopper/fuel",fuelInHopper);
    SmartDashboard.putBoolean("hopper/isLow",isLow.getAsBoolean());
    SmartDashboard.putBoolean("hopper/isFull",isFull.getAsBoolean());
    SmartDashboard.putBoolean("hopper/isNotFull",isNotFull.getAsBoolean());
  }  

  private void setupRealTriggers(){
    if(Robot.isReal()==false) return;
    //TODO: The real bot can use Lasercan to determine imprecise levels
    // or approximate counts, but not exact ones. The triggers may differ
    // Or we can just write approximate fuel state based off the scans
  }


  private void setupSimulationTriggers(){
    if(Robot.isSimulation()==false) return;
    //These interplay with the FuelSim and can have an exact fuel count,
    //But  we cannot effectively simulate lasercan/ imprecise levels
    isLow = new Trigger(()->fuelInHopper<=kMaxCapacity);
    isFull = new Trigger(()->fuelInHopper>=kMaxCapacity);
    isNotFull = new Trigger(()->fuelInHopper<kMaxCapacity-4);
  }

}
