package frc.robot.Subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.Shooter.Feeder.Feeder;
import frc.robot.Subsystems.Shooter.Flywheel.Flywheel;
import frc.robot.Subsystems.Shooter.Hood.Hood;
import frc.robot.Subsystems.Shooter.Turret.Turret;

public class Shooter {
    Feeder feeder = new Feeder();
    Flywheel flywheel = new Flywheel();
    Turret turret = new Turret();
    Hood hood = new Hood();

    /** Just set up the mechanism2d so we can visualize the system all at once */
    ShooterVisual visual = new ShooterVisual(feeder, flywheel, hood, turret);

    //TODO create helpful commands and/or logic
    //Note, this is not a subsystem, but we can turn it into one
    //There's some considerations in doing so worth working through

    //TODO: Sync this method/concept with shooter code
    public Trigger isReadyToAcceptFuel = new Trigger(()->true);

}
