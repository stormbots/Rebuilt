package frc.robot.Subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Shooter.Feeder.Feeder;
import frc.robot.Subsystems.Shooter.Flywheel.Flywheel;
import frc.robot.Subsystems.Shooter.Flywheel.Flywheel.WantedState;
import frc.robot.Subsystems.Shooter.Hood.Hood;
import frc.robot.Subsystems.Shooter.Turret.Turret;

public class Shooter {
    Feeder feeder = new Feeder();
    Flywheel flywheel = new Flywheel();
    Turret turret = new Turret();
    Hood hood = new Hood();

    private double targetRPM = 4000.0;

    /** Just set up the mechanism2d so we can visualize the system all at once */
    ShooterVisual visual = new ShooterVisual(feeder, flywheel, hood, turret);

    //TODO create helpful commands and/or logic
    //Note, this is not a subsystem, but we can turn it into one
    //There's some considerations in doing so worth working through

    public Command shoot(){
        //this whole thing is pretty temporary, whole system is going to be reorganized after flywheel testing
        return new RunCommand(()->flywheel.setWantedState(WantedState.SETRPM, targetRPM)).finallyDo(()->flywheel.setWantedState(WantedState.STOP));
    }

}
