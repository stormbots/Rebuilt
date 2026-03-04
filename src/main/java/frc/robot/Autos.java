// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import java.util.concurrent.CompletableFuture;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Questnav.QuestNavSubsystem;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Spindexer.Spindexer;
import frc.robot.Subsystems.Swerve.Pathing;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.TargetingSystem.TargetingSystem;
import frc.robot.lib.BLine.Path;

/** Add your docs here. */
public class Autos {

    Swerve swerve;
    Shooter shooter;
    Intake intake;
    QuestNavSubsystem questNav;
    Spindexer spindexer;
    Pathing pathing;
    TargetingSystem targeting;

    SendableChooser<Supplier<Command>> autoChooser = new SendableChooser<>();
    private CompletableFuture<Command> selectedAutoFuture = CompletableFuture.supplyAsync(()->new InstantCommand());

    public Autos(
        Swerve swerve,
        Shooter shooter,
        Intake intake,
        QuestNavSubsystem questNav,
        Spindexer spindexer,
        Pathing pathing,
        TargetingSystem targeting
    ){
        this.swerve = swerve;
        this.shooter = shooter;
        this.intake = intake;
        this.questNav = questNav;
        this.spindexer = spindexer;
        this.pathing = pathing;
        this.targeting = targeting;

        SmartDashboard.putData("AutoSelector/chooser",autoChooser);
        autoChooser.setDefaultOption("Select Auto",()->new InstantCommand());

        //ACTUAL OPTIONS BELOW HERE
        autoChooser.addOption("THE ORIGINAL PATHING AUTO", this::ogAutoHeHeHe);
    }


    //Get Auto Command
    public Command getAutonomousCommand(){
        // try{
        //      return selectedAutoFuture.get();
        // }
        // catch(Exception e){
        //     System.err.println("Failed to build auto command ");
        //     System.err.println(e);
        // }
        // return new InstantCommand();
        return ogAutoHeHeHe();
    }


    /////////////////////////
    //ALL AUTOS BELLOW HERE//
    /////////////////////////
    public Command ogAutoHeHeHe(){
        //This is kind of basic format, we will want to run sequences/parallel groups and stuff once shooter and allat are up
        return Commands.sequence(
            Commands.print("1"),
            swerve.turnToHeading(()->{
                return targeting.getHeadingToTarget(swerve.getSwervePose().getTranslation(), targeting.getHubTarget()).plus(Rotation2d.k180deg);
            }).withTimeout(2.0),
            Commands.print("2"),
            new ParallelCommandGroup(
                shooter.shootHubAuto(),
                new WaitCommand(2.0)
                .andThen(spindexer.feedToShooterForce())
            ).withTimeout(8.0),
            Commands.print("3"),
            shooter.testSetHoodAngle(Degrees.of(0)).withTimeout(0.5),
            new ParallelCommandGroup(
                pathing.followPath(new Path("orbitAHHH")),
                intake.intake().withTimeout(5.0)
            ),
            swerve.turnToHeading(()->{
                return targeting.getHeadingToTarget(swerve.getSwervePose().getTranslation(), targeting.getHubTarget()).plus(Rotation2d.k180deg);
            }).withTimeout(2.0),
            Commands.print("2"),
            new ParallelCommandGroup(
                shooter.shootHubAuto(),
                new WaitCommand(2.0)
                .andThen(spindexer.feedToShooterForce())
            ).withTimeout(8.0)
        );
    }
    


}