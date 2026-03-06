// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import java.util.concurrent.CompletableFuture;
import java.util.function.Supplier;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    // private CompletableFuture<Command> selectedAutoFuture = CompletableFuture.supplyAsync(()->new InstantCommand());

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


        autoChooser.addOption("VV UNTESTED VV",()->new InstantCommand());

        //ACTUAL OPTIONS BELOW HERE
        autoChooser.addOption("THE ORIGINAL PATHING AUTO", this::centerShootRightBlue);
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
        return autoChooser.getSelected().get();
    }


    /////////////////////////
    //ALL AUTOS BELLOW HERE sk was here!!!!!!!!!//
    /////////////////////////
    public Command basicShootInitial8(){
        return Commands.sequence(
            swerve.turnToHeading(()->{
                return targeting.getHeadingToTarget(swerve.getSwervePose().getTranslation(), targeting.getHubTarget()).plus(Rotation2d.k180deg);
            }).until(()->swerve.isOnTargetAngle()).withTimeout(1.0),
            new ParallelCommandGroup(
                shooter.shootHubAuto(),
                new WaitCommand(0.3)
                .andThen(spindexer.feedToShooterForce())
            ).withTimeout(1.5)
        );
    }
    public Command centerShootRightBlue(){
        Path.PathConstraints constraints = new Path.PathConstraints()
            .setMaxVelocityMetersPerSec(4.0)
            .setMaxAccelerationMetersPerSec2(4.0)
            .setMaxVelocityDegPerSec(360.0)
            .setMaxAccelerationDegPerSec2(580.0)
            .setEndTranslationToleranceMeters(0.4)
            .setEndRotationToleranceDeg(5.0);
          Path pointStart = new Path(
            constraints,
            new Path.Waypoint(new Translation2d(4.0, 0.8), new Rotation2d(1.5707963267948966))
        );
        Path pathInt = new Path(
            constraints,
            new Path.Waypoint(new Translation2d(7.7, 0.8), new Rotation2d(1.5707963267948966)),
            new Path.Waypoint(new Translation2d(7.7, 2.5), new Rotation2d(1.5707963267948966)),
            new Path.Waypoint(new Translation2d(7.7, 0.8), new Rotation2d(1.5707963267948966))
        );
        Path pointPostInt = new Path(
            constraints,
            new Path.Waypoint(new Translation2d(7.7, 0.8), new Rotation2d(1.5707963267948966)),
            new Path.Waypoint(new Translation2d(4.0, 0.8), new Rotation2d(1.5707963267948966))
        );
        return Commands.sequence(
            Commands.print("1"),
            basicShootInitial8(),
            Commands.print("3"),
            shooter.testSetHoodAngle(Degrees.of(0)).withTimeout(0.5),
            Commands.print("4"),
            pathing.followPath(pointStart).withTimeout(0.3),
            Commands.print("5"),
            pathing.followPath(pathInt).withTimeout(10.0),
            Commands.print("8"),
            intake.stop().withTimeout(0.2),
            Commands.print("9"),
            pathing.followPath(pointPostInt).withTimeout(2.5),
            Commands.print("10"),
            basicShootInitial8()
        );
    }
    public Command centerShootLeftBlue(){
        Path pointStart = new Path(
            new Path.Waypoint(new Translation2d(4.0, 0.8), new Rotation2d(0))
        );
        Path pointPreInt = new Path(
            new Path.Waypoint(new Translation2d(7.7, 0.8), new Rotation2d(0))
        );
        Path pointPostInt = new Path(
            new Path.Waypoint(new Translation2d(7.7, 2.5), new Rotation2d(0))
        );
        return Commands.sequence(
            Commands.print("1"),
            basicShootInitial8(),
            Commands.print("3"),
            shooter.testSetHoodAngle(Degrees.of(0)).withTimeout(0.5),
            pathing.followPath(pointStart),
            pathing.followPath(pointPreInt),
            pathing.followPath(pointPostInt).alongWith(intake.intake()),
            pathing.followPath(pointPreInt),
            intake.stop(),
            pathing.followPath(pointStart),
            basicShootInitial8()
        );
    }
    public Command centerShootRightRed(){
        return Commands.sequence(
            Commands.print("1"),
            basicShootInitial8(),
            Commands.print("3"),
            shooter.testSetHoodAngle(Degrees.of(0)).withTimeout(0.5),
            new ParallelCommandGroup(
                pathing.followPathTeamFlipped((new Path("orbitAHHH"))),
                intake.intake().withTimeout(5.0)
            ),
            basicShootInitial8()
        );
    }
    public Command centerShootLeftRed(){
        return Commands.sequence(
            Commands.print("1"),
            basicShootInitial8(),
            Commands.print("3"),
            shooter.testSetHoodAngle(Degrees.of(0)).withTimeout(0.5),
            new ParallelCommandGroup(
                pathing.followPath(new Path("orbitAHHH")),
                intake.intake().withTimeout(5.0)
            ),
            basicShootInitial8()
        );
    }

    public Command pidNotBlineCenterShoot(){
        return Commands.sequence(
            basicShootInitial8(),
            shooter.testSetHoodAngle(Degrees.of(0)).withTimeout(0.5),
            swerve.pidToPose(new Pose2d(7.7, 0.8, new Rotation2d(1.5707963267948966))).until(()->(swerve.isOnTargetTranslate() && swerve.isOnTargetAngle())),
            swerve.pidToPose(new Pose2d(7.7, 2.5, new Rotation2d(1.5707963267948966))).until(()->(swerve.isOnTargetTranslate() && swerve.isOnTargetAngle())),
            swerve.pidToPose(new Pose2d(7.7, 0.8, new Rotation2d(1.5707963267948966))).until(()->(swerve.isOnTargetTranslate() && swerve.isOnTargetAngle())),
            swerve.pidToPose(new Pose2d(4.0, 0.8, new Rotation2d(1.5707963267948966))).until(()->(swerve.isOnTargetTranslate() && swerve.isOnTargetAngle()))
        );
    }
    


}