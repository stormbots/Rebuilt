// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.HopperSensors;

import static edu.wpi.first.units.Units.Inches;

import com.stormbots.LaserCanWrapper;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;

public class HopperSensors extends SubsystemBase {
  private static HopperSensors instance;
  // Readings above these values indicate a new fuel state
  public final int kMaxSensorDistance = 13; //inches
  public final int kEmpty = 4; 
  public final int kLowCapacity = 6;
  public final int kMaxCapacity = 10;
  private LaserCanWrapper[] laserCan = new LaserCanWrapper[]{new LaserCanWrapper(30), new LaserCanWrapper(31), new LaserCanWrapper(32), new LaserCanWrapper(33)};
  /** Most fuel capacity is spent, and we should consider doing something else */
  public Trigger isLow = new Trigger(this::isLow);
  /** At maximum capacity, and should not attempt to pull in more */
  public Trigger isFull = new Trigger(this::isFull);
  /** Fuel has room to continue intaking */
  public Trigger isPartiallyLoaded = new Trigger(this::isPartiallyLoaded);

  /** Creates a new HopperSensors. */
  private HopperSensors(){
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
    SmartDashboard.putBoolean("hopper/isLow",isLow.getAsBoolean());
    SmartDashboard.putBoolean("hopper/isFull",isFull.getAsBoolean());
    SmartDashboard.putBoolean("hopper/isNotFull",isPartiallyLoaded.getAsBoolean());

    // plotDetailedSensorInfo(); //For diagnostics
    if(Robot.isSimulation()){
      SmartDashboard.putNumber("hopper/fuel",simFuelInHopper);
    }
  }  

  public Boolean isFull(){
    return getLayer(kMaxCapacity);
  }

  public Boolean isPartiallyLoaded(){
    return getLayer(kLowCapacity);
  }

  public boolean isLow(){
    return getLayer(kEmpty);
  }

  private boolean getLayer(int layerHeight){
    int full = 0;
    for(int i = 0; i < laserCan.length; i++){
      if(laserCan[i].getDistanceOptional().orElse(Inches.of(kMaxSensorDistance)).in(Inches) >13 - layerHeight){
        full++;
      }
    }
    if(full >= 3){
      return true;
    } else {
      return false;
    }

  }

  private void plotDetailedSensorInfo(){
    double min = kMaxSensorDistance;
    double max = 0;
    double sum = 0;

    for(var lc : laserCan){
      var reading = lc.getDistanceOptional().orElse(Inches.of(kMaxSensorDistance)).in(Inches);
      min = reading<min ? reading : min;
      max = reading>max ? reading : max;
      sum += reading;
    }

    SmartDashboard.putNumber("hoppersensors/data/min",min);
    SmartDashboard.putNumber("hoppersensors/data/max",max);
    SmartDashboard.putNumber("hoppersensors/data/sum",sum);
    SmartDashboard.putNumber("hoppersensors/data/mean",sum/4.0);

    SmartDashboard.putBoolean("hoppersensors/data/low",isLow.getAsBoolean());
    SmartDashboard.putBoolean("hoppersensors/data/low",isPartiallyLoaded.getAsBoolean());
    SmartDashboard.putBoolean("hoppersensors/data/full",isFull.getAsBoolean());

  }

  private void setupRealTriggers(){
    if(Robot.isReal()==false) return;
    isLow = new Trigger(this::isLow);
    isFull = new Trigger(this::isFull);
    isPartiallyLoaded = new Trigger(this::isPartiallyLoaded);
  }


  public int simFuelInHopper = 8; // Initial fuel provided during autos; Only useful for sim
  private void setupSimulationTriggers(){
    if(Robot.isSimulation()==false) return;
    //These interplay with the FuelSim and can have an exact fuel count,
    //But  we cannot effectively simulate lasercan/ imprecise levels
    isLow = new Trigger(()->simFuelInHopper<=2);
    isFull = new Trigger(()->simFuelInHopper>=10);
    isPartiallyLoaded = new Trigger(()->simFuelInHopper<6);
  }

}
