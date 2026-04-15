package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.Subsystems.HopperSensors.HopperSensors;
import frc.robot.Subsystems.HopperSensors.FuelSim.FuelSim;
import frc.robot.Subsystems.Shooter.Flywheel.Flywheel;
import frc.robot.Subsystems.Shooter.Hood.Hood;
import frc.robot.Subsystems.Shooter.Turret.Turret;
import frc.robot.Subsystems.Shooter.Turret.TurretVisual;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.TargetingSystem.TargetingSystem;

public class Shooter {
  Swerve swerve;
  Flywheel flywheel = new Flywheel();
  Hood hood = new Hood(new Trigger(()->(
    (swerve.getSwervePose().getX() >= 4.3 && swerve.getSwervePose().getX() <= 5.08) 
    || (swerve.getSwervePose().getX() >= 11.47 && swerve.getSwervePose().getX() <= 12.43))));
  Turret turret = new Turret();
  TargetingSystem targeting;

  /**
   * Represent the hood being suppressed for the trench, and block fuel in that
   * case
   */
  private boolean stowed = false;

  // TODO: Sync this method/concept with shooter code
  public Trigger isReadyToAcceptFuel = new Trigger(() -> flywheel.getOnTarget() && hood.getOnTarget() && turret.getOnTarget() && stowed == false).debounce(0.1);

  /** Just set up the mechanism2d so we can visualize the system all at once */
  // ShooterVisual visual = new ShooterVisual(flywheel, hood, turret);
  TurretVisual visual = new TurretVisual(turret);
  Trigger visualUpdater = new Trigger(DriverStation::isEnabled).onTrue(
      Commands.run(() -> visual.update(turret.getAngle())));

  public Shooter(TargetingSystem targeting, Swerve swerve) {
    this.swerve = swerve;
    this.targeting = targeting;
    //TODO: Enable once we're happy with the turret not jamming
    // turret.setDefaultCommand(turret.setAngle(targeting::getTurretTracking));
  }

  public Command shoot(Supplier<TargetingSystem.ShooterState> targets) {
    return Commands.parallel(
      flywheel.setRPM(targets),
      hood.setAngle(targets),
      
      // turret.setAngle(targets)
      //TODO: fix settrap so we can use it instead.
      turret.setAngleTrap(targets) 
    );
  }

  public Command shootNoTurret(Supplier<TargetingSystem.ShooterState> targets) {
    return Commands.parallel(
      flywheel.setRPM(targets),
      hood.setAngle(targets),
      turret.setAngle(()->Degrees.of(-180.0), ()->Degrees.of(5.0))
    );
  }

  public Command testSetTurretAngle(Angle angle) {
    return turret.setAngle(()->angle, ()->Degrees.of(3));
  }

  public Command testSetHoodAngle(Angle angle) {
    return hood.setAngle(()->angle, ()->Degrees.of(3));
  }

  public Command testSetFlywheelRPM(double rpm) {
    return flywheel.setRPM(()->rpm, ()->300);
  }

  public Command testHome() {
    return hood.homingCommand();
  }

  public Command simGetLaunchCommand() {
    // Don't do anything on a normal bot
    if (Robot.isReal()){
      return Commands.idle();
    }

    double fuelPerSecond = 8;

    var shot = Commands.runOnce(() -> {
      if (HopperSensors.getInstance().fuelInHopper <= 0){
        return;
      }

      HopperSensors.getInstance().fuelInHopper--;

      var initialPosition = targeting.getTurretCenterpoint();
      var velocity = targeting.simGenerateIdealShot();

      FuelSim.getInstance().spawnFuel(initialPosition, velocity);
    });

    return Commands.sequence(
      shot,
      Commands.waitSeconds(1 / fuelPerSecond))
      .repeatedly();
  }

  public double getFlywheelRpm(){
    return flywheel.getRPM();
  }

  public Command shootHub() {
    return shoot(targeting::getHub);
  }

  public Command shootHubNoTur() {
    return shootNoTurret(targeting::getHubBotVelCompensated);
  }

  public Command shootHubVelComp() {
    return shoot(targeting::getHubBotVelCompensated);
  }

  public Command pass() {
    return shoot(targeting::getPassBotVelCompensated);
  }

  /** Bring the hood down for trench purposes */
  public Command stow() {
    return Commands.parallel(
      hood.stow()
    )
    .beforeStarting(()->stowed = true)
    .finallyDo(()->stowed = false);
  }

  public Command testTurretVoltage(DoubleSupplier voltage) {
    return turret.setVoltage(voltage);
  }

  // For tuning LUTs, read
  public Command shootWithDashboardValues() {
    // return Commands.none();
    // SmartDashboard.putNumber("robotContainer/hoodAngle", 0);
    // SmartDashboard.putNumber("robotContainer/flywheelrpm", 0);
    return this.shoot(() -> new TargetingSystem.ShooterState(
      Degrees.of(-180),
      Degrees.of(SmartDashboard.getNumber("robotContainer/hoodAngle", hood.getAngle().in(Degrees))),
      SmartDashboard.getNumber("robotContainer/flywheelrpm", flywheel.getRPM()))
    );
  }

}
