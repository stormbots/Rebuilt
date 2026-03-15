// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meter;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Bumpers;
import frc.robot.Constants.Intake;
import frc.robot.Subsystems.HopperSensors.HopperSensors;
import frc.robot.Subsystems.HopperSensors.FuelSim.FuelSim;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer robotContainer;

  public Robot() {
    robotContainer = new RobotContainer();
    //Set up the GrappleHook bridge application
    CanBridge.runTCP();
    PortForwarder.add(5800, "photonvision.local", 5800);

    switch(Preferences.getString("BotName", "compbot")){
      case "tabi":
      //Set up a network bridge to access the Orange Pi
      // PortForwarder.add(5800, "photonvision.local", 5800);
      break;

      case "practicebot":
      break;

      case "compbot":

      break;

      case "compBot":
        //Actually reset the name.
        Preferences.setString("BotName", "compbot");
      default:
      //No name set: Set a default to make the key visible.
      Preferences.setString("BotName", "compbot");
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    robotContainer.syncQuestPose();
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
   m_autonomousCommand = robotContainer.autos.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationInit() {
    super.simulationInit();
    if(DriverStation.isDisabled()) return;
    setupFuelSimulation();
  }

  @Override
  public void simulationPeriodic() {
    super.simulationInit();
    //Simulate stuff for a bit then stop because it burns battery
    if(Timer.getFPGATimestamp()<=120){
      FuelSim.getInstance().updateSim();
    }else if(Timer.getFPGATimestamp()<=121){
      FuelSim.getInstance().clearFuel();
    }
  }

  /** Configure the fuel simulation. Does nothing if not in simulation. */
  public void setupFuelSimulation(){
    if(Robot.isSimulation()==false)return;
    FuelSim.getInstance(); // gets singleton instance of FuelSim
    FuelSim.getInstance().spawnStartingFuel(); // spawns fuel in the depots and neutral zone

    // Register a robot for collision with fuel
    FuelSim.getInstance().registerRobot(
      Bumpers.width.in(Meter), // from left to right
      Bumpers.length.in(Meter), // from front to back
      Bumpers.height.in(Meter), // from floor to top of bumpers
      robotContainer.swerve::getSwervePose, // Supplier<Pose2d> of robot pose
      robotContainer.swerve::getChassisSpeeds // Supplier<ChassisSpeeds> of field-centric chassis speeds
    );

    // Register an intake to remove fuel from the field as a rectangular bounding box
    FuelSim.getInstance().registerIntake(
      //Robotcentric X coordinates of the BB
      Bumpers.length.div(2.0).in(Meter), 
      Bumpers.length.div(2.0).plus(Intake.reach).in(Meter), 
      //Y Coordinates of the intake bounding box
      Intake.width.div(-2.0).in(Meter), 
      Intake.width.div(2.0).in(Meter),
      robotContainer.intake.isDeployed, // (optional) BooleanSupplier for whether the intake should be active at a given moment
      ()->HopperSensors.getInstance().simFuelInHopper++ // (optional) Runnable called whenever a fuel is intaked
    ); 

    FuelSim.getInstance().setSubticks(5); // sets the number of physics iterations to perform per 20ms loop. Default = 5

    FuelSim.getInstance().start(); // enables the simulation to run (updateSim must still be called periodically)
    // FuelSim.getInstance().stop(); // stops the simulation running (updateSim will do nothing until start is called again)

    SmartDashboard.putData("hopper/ResetFieldFuel",Commands.runOnce(() -> {
      FuelSim.getInstance().clearFuel();
      FuelSim.getInstance().spawnStartingFuel();
    })
    .withName("Reset Fuel")
    .ignoringDisable(true));
  }
}
