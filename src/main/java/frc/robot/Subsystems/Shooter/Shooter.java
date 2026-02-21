package frc.robot.Subsystems.Shooter;

import com.stormbots.LUT;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    public Command shoot(TargetingSystem.ShooterState targets){
        return Commands.parallel(
            flywheel.setRPMCommand(targets.flywheelRPM, targets.flywheelTolerance),
            hood.setAngleCommand(targets.hoodAngle, targets.hoodTolerance),
            turret.setAngleCommand(targets.turretAngle, targets.turretTolerance)
        );
    }

    public Command constantVoltage(double volts){
        return flywheel.setVoltageCommand(volts);
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

    //setTurretAngle independently
    //setHood Angle indepenedenyt
    //setflywheel rpm

    public Command shootHub(){
        return shoot(targeting.getShotForHub());
    }
    public Command pass(){
        return shoot(targeting.getPass());
    }

    public Command doTheObviousThingDriversWant(){
        var target = targeting.getBestTarget();
        targeting.getShotForHub();
        targeting.getPass();

        // return shoot(ShooterState);
        return Commands.none();
    }




}
