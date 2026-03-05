package frc.robot.Subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;

import java.lang.annotation.Target;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.Subsystems.TargetingSystem.TargetingSystem;

public class Shooter {
Flywheel flywheel = new Flywheel();
    Turret turret = new Turret();
    Hood hood = new Hood();

    TargetingSystem targeting;

    //TODO: Sync this method/concept with shooter code
    public Trigger isReadyToAcceptFuel = new Trigger(()->flywheel.getOnTarget() && hood.getOnTarget() && turret.getOnTarget()).debounce(0.05);


    /** Just set up the mechanism2d so we can visualize the system all at once */
    // ShooterVisual visual = new ShooterVisual(flywheel, hood, turret);
    TurretVisual visual = new TurretVisual(turret);
    Trigger visualUpdater = new Trigger(DriverStation::isEnabled).onTrue(
        Commands.run(()->visual.update(turret.getAngle()))
    );


    //TODO create helpful commands and/or logic
    //Note, this is not a subsystem, but we can turn it into one
    //There's some considerations in doing so worth working through


    public Shooter(TargetingSystem targeting) {
        this.targeting=targeting;
    }

    // public TargetingSystem.ShooterState getCurrentState(){
    //     return new TargetingSystem.ShooterState(turret.getAngle(), hood.getAngle(), flywheel.getRPM());
    // }

    public Command shoot(Supplier<TargetingSystem.ShooterState> targets){
        return Commands.parallel(
            flywheel.setRPM(targets),
            hood.setAngle(targets),
            turret.setAngle(targets)
        );
    }
    public Command shootNoTurret(Supplier<TargetingSystem.ShooterState> targets){
        return Commands.parallel(
            flywheel.setRPM(targets),
            hood.setAngle(targets),
            turret.setAngle(()->Degrees.of(180.0), ()->Degrees.of(5.0))
        );
    }

    public Command testSetTurretAngle(Angle angle){
        return turret.setAngle(()->angle, ()->Degrees.of(3));
    }

    public Command testSetHoodAngle(Angle angle){
        return hood.setAngle(()->angle, ()->Degrees.of(3));
    }

    public Command testSetFlywheelRPM(double rpm){
        return flywheel.setRPM(()->rpm, ()->300);
    }
    
    // public Command testFlywheelVoltage(double volts){
    //     return flywheel.setVoltageCommand(volts);
    // }

    public Command testHome(){
        return hood.homingCommand();
    }

    public Command simGetLaunchCommand(){
        //Don't do anything on a normal bot
        if(Robot.isReal())return Commands.idle();

        double fuelPerSecond=8;

        var shot=Commands.runOnce(()->{
            if (HopperSensors.getInstance().fuelInHopper <= 0) return;
            HopperSensors.getInstance().fuelInHopper--;

            var initialPosition = targeting.getTurretCenterpoint();
            var velocity=targeting.simGenerateIdealShot();

            FuelSim.getInstance().spawnFuel(initialPosition, velocity);
        });

        return Commands.sequence(
        shot,
        Commands.waitSeconds(1/fuelPerSecond)
        )
        .repeatedly();
    }

    public Command shootHub(){
        return shoot(targeting::getHub);
    }
    public Command shootHubAuto(){
        return shootNoTurret(targeting::getHub);
    }

    public Command pass(){
        return shoot(targeting::getPass);
    }

    // For tuning LUTs, read 
    public Command shootWithDashboardValues(){
        return Commands.none();
    // return shooter.shoot(()->new TargetingSystem.ShooterState(
    //   Degrees.of(0), 
    //   Degrees.of(SmartDashboard.getNumber("robotContainer/hoodAngle", hoodAngle)), 
    //   SmartDashboard.getNumber("robotContainer/flywheelrpm", rpm)))
    // );
    }

    public Command doTheObviousThingDriversWant(){
        Supplier<TargetingSystem.ShooterState> bestState = ()->{

            //some logic. not in.
            var target = targeting.getBestTarget();
            targeting.getHub();
            return targeting.getPass();
            
        };

        return shoot(bestState);
    }
    
}
