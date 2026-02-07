// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.CompletableFuture;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Questnav.QuestNavSubsystem;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Spindexer.Spindexer;
import frc.robot.Subsystems.Swerve.Pathing;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.lib.BLine.Path;

/** Add your docs here. */
public class Autos {

    Swerve swerve;
    Shooter shooter;
    Intake intake;
    QuestNavSubsystem questNav;
    Spindexer spindexer;
    Pathing pathing;

    SendableChooser<Supplier<Command>> autoChooser = new SendableChooser<>();
    private CompletableFuture<Command> selectedAutoFuture = CompletableFuture.supplyAsync(()->new InstantCommand());

    public Autos(
        Swerve swerve,
        Shooter shooter,
        Intake intake,
        QuestNavSubsystem questNav,
        Spindexer spindexer,
        Pathing pathing
    ){
        this.swerve = swerve;
        this.shooter = shooter;
        this.intake = intake;
        this.questNav = questNav;
        this.spindexer = spindexer;
        this.pathing = pathing;

        SmartDashboard.putData("AutoSelector/chooser",autoChooser);
        autoChooser.setDefaultOption("Select Auto",()->new InstantCommand());

        //ACTUAL OPTIONS BELOW HERE
        autoChooser.addOption("THE ORIGINAL PATHING AUTO", this::ogAutoHeHeHe);
    }


    //Get Auto Command
    public Command getAutonomousCommand(){
        try{
             return selectedAutoFuture.get();
        }
        catch(Exception e){
            System.err.println("Failed to build auto command ");
            System.err.println(e);
        }
        return new InstantCommand();
    }


    /////////////////////////
    //ALL AUTOS BELLOW HERE//
    /////////////////////////
    public Command ogAutoHeHeHe(){
        //This is kind of basic format, we will want to run sequences/parallel groups and stuff once shooter and allat are up
        return pathing.followPath(new Path("goCollect"));
    }
    


}
