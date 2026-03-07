// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import java.util.concurrent.CompletableFuture;
import java.util.function.Supplier;
import javax.sound.midi.Sequence;
import org.opencv.core.Mat;
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
    //Starting poses
        Pose2d startPoseBL = new Pose2d(4.0, 7.4, new Rotation2d());
        Pose2d startPoseBR = new Pose2d(4.0, 0.6, new Rotation2d());
        Pose2d startPoseRL = new Pose2d(12.5, 0.6, new Rotation2d());
        Pose2d startPoseRR = new Pose2d(12.5, 7.4, new Rotation2d());
       
        //Pre intaking poses
        Pose2d preIntBL = new Pose2d(7.7, 7.4, new Rotation2d(-Math.PI/2));
        Pose2d preIntBR = new Pose2d(7.7, 0.6, new Rotation2d(Math.PI/2));
        Pose2d preIntRL = new Pose2d(8.85, 0.6, new Rotation2d(Math.PI/2));
        Pose2d preIntRR = new Pose2d(8.85, 7.4, new Rotation2d(-Math.PI/2));
       
        //Post intaking poses
        Pose2d postIntBL = new Pose2d(7.7, 5.5, new Rotation2d(-Math.PI/2));
        Pose2d postIntBR = new Pose2d(7.7, 2.5, new Rotation2d(Math.PI/2));
        Pose2d postIntRL = new Pose2d(8.85, 2.5, new Rotation2d(Math.PI/2));
        Pose2d postIntRR = new Pose2d(8.85, 5.5, new Rotation2d(-Math.PI/2));
       
        //Final poses
        Pose2d shotPoseBL = new Pose2d(3.6, 7.4, new Rotation2d());
        Pose2d shotPoseBR = new Pose2d(3.6, 0.6, new Rotation2d());
        Pose2d shotPoseRL = new Pose2d(13.5, 0.6, new Rotation2d());
        Pose2d shotPoseRR = new Pose2d(13.5, 7.4, new Rotation2d());

        //Depot poses
        Pose2d preIntDepotBlue = new Pose2d(1.0, 6.0, new Rotation2d(Math.PI));
        Pose2d preIntDepotRed = new Pose2d(15.5, 2.0, new Rotation2d());
        Pose2d intDepotBlue = new Pose2d(0.6, 6.0, new Rotation2d(Math.PI));
        Pose2d intDepotRed = new Pose2d(15.9, 2.0, new Rotation2d());
        

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
        autoChooser.addOption("Basic Shoot 8 anywhere", this::basicShootInitial8);
        autoChooser.addOption("Right Blue go center", this::centerShootRightBlue);
        autoChooser.addOption("Right Red go center", this::centerShootRightBlue);
        autoChooser.addOption("Left Blue go center", this::centerShootLeftBlue);
        autoChooser.addOption("Left Red go center", this::centerShootLeftRed);
        autoChooser.addOption("Left Blue go center not BLINE", this::notBlineCenterBlueLeftShoot);
        autoChooser.addOption("Right Blue go center not BLINE", this::notBlineCenterBlueRightShoot);
        autoChooser.addOption("Left Red go center not BLINE", this::notBlineCenterRedLeftShoot);
        autoChooser.addOption("Right Red go center not BLINE", this::notBlineCenterRedRightShoot);
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
    public Command basicShootToEmpty(){
        return Commands.sequence(
            swerve.turnToHeading(()->{
                return targeting.getHeadingToTarget(swerve.getSwervePose().getTranslation(), targeting.getHubTarget()).plus(Rotation2d.k180deg);
            }).until(()->swerve.isOnTargetAngle()).withTimeout(1.0),
            new ParallelCommandGroup(
                shooter.shootHubAuto(),
                new WaitCommand(0.3)
                .andThen(spindexer.feedToShooterForce())
            ).withTimeout(3)
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
        Path.PathConstraints constraints2 = new Path.PathConstraints()
            .setMaxVelocityMetersPerSec(4.0)
            .setMaxAccelerationMetersPerSec2(4.0)
            .setMaxVelocityDegPerSec(360.0)
            .setMaxAccelerationDegPerSec2(580.0)
            .setEndTranslationToleranceMeters(0.4)
            .setEndRotationToleranceDeg(5.0);
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
            new ParallelCommandGroup(
                pathing.followPath(pathInt).withTimeout(15.0),
                Commands.sequence(
                    new WaitCommand(2.0),
                    intake.intake())),
            Commands.print("8"),
            intake.stop().withTimeout(0.2),
            Commands.print("9"),
            pathing.followPath(pointPostInt).withTimeout(7.5),
            Commands.print("10"),
            basicShootInitial8()
        );
    }
    public Command centerShootLeftBlue(){
        Path.PathConstraints constraints = new Path.PathConstraints()
            .setMaxVelocityMetersPerSec(4.0)
            .setMaxAccelerationMetersPerSec2(4.0)
            .setMaxVelocityDegPerSec(360.0)
            .setMaxAccelerationDegPerSec2(580.0)
            .setEndTranslationToleranceMeters(0.4)
            .setEndRotationToleranceDeg(5.0);
          Path pointStart = new Path(
            constraints,
            new Path.Waypoint(new Translation2d(4.0, 7.4), new Rotation2d(-1.5707963267948966))
        );
        Path pathInt = new Path(
            constraints,
            new Path.Waypoint(new Translation2d(7.7, 7.4), new Rotation2d(-1.5707963267948966)),
            new Path.Waypoint(new Translation2d(7.7, 5.0), new Rotation2d(-1.5707963267948966)),
            new Path.Waypoint(new Translation2d(7.7, 7.4), new Rotation2d(-1.5707963267948966))
        );
        Path pointPostInt = new Path(
            constraints,
            new Path.Waypoint(new Translation2d(7.7, 7.4), new Rotation2d(-1.5707963267948966)),
            new Path.Waypoint(new Translation2d(4.0, 7.4), new Rotation2d(-1.5707963267948966))
        );
        return Commands.sequence(
            Commands.print("1"),
            basicShootInitial8(),
            Commands.print("3"),
            shooter.testSetHoodAngle(Degrees.of(0)).withTimeout(0.5),
            Commands.print("4"),
            new ParallelCommandGroup(
                pathing.followPath(pathInt).withTimeout(15.0),
                Commands.sequence(
                    new WaitCommand(2.0),
                    intake.intake())),
            Commands.print("8"),
            intake.stop().withTimeout(0.2),
            Commands.print("9"),
            pathing.followPath(pointPostInt).withTimeout(7.5),
            Commands.print("10"),
            basicShootInitial8()
        );
    }
    public Command centerShootRightRed(){
         Path.PathConstraints constraints = new Path.PathConstraints()
            .setMaxVelocityMetersPerSec(4.0)
            .setMaxAccelerationMetersPerSec2(4.0)
            .setMaxVelocityDegPerSec(360.0)
            .setMaxAccelerationDegPerSec2(580.0)
            .setEndTranslationToleranceMeters(0.4)
            .setEndRotationToleranceDeg(5.0);
          Path pointStart = new Path(
            constraints,
            new Path.Waypoint(new Translation2d(4.0, 0.6), new Rotation2d(-1.5707963267948966))
        );
        Path pathInt = new Path(
            constraints,
            new Path.Waypoint(new Translation2d(7.7, 0.6), new Rotation2d(-1.5707963267948966)),
            new Path.Waypoint(new Translation2d(7.7, 2.5), new Rotation2d(-1.5707963267948966)),
            new Path.Waypoint(new Translation2d(7.7, 0.6), new Rotation2d(-1.5707963267948966))
        );
        Path pointPostInt = new Path(
            constraints,
            new Path.Waypoint(new Translation2d(7.7, 0.6), new Rotation2d(-1.5707963267948966)),
            new Path.Waypoint(new Translation2d(4.0, 0.6), new Rotation2d(-1.5707963267948966))
        );
        return Commands.sequence(
            Commands.print("1"),
            basicShootInitial8(),
            Commands.print("3"),
            shooter.testSetHoodAngle(Degrees.of(0)).withTimeout(0.5),
            Commands.print("4"),
            new ParallelCommandGroup(
                pathing.followPathTeamFlipped(pathInt).withTimeout(15.0),
                Commands.sequence(
                    new WaitCommand(2.0),
                    intake.intake())),
            Commands.print("8"),
            intake.stop().withTimeout(0.2),
            Commands.print("9"),
            pathing.followPathTeamFlipped(pointPostInt).withTimeout(7.5),
            Commands.print("10"),
            basicShootInitial8()
        );
    }
    public Command centerShootLeftRed(){
        Path.PathConstraints constraints = new Path.PathConstraints()
            .setMaxVelocityMetersPerSec(4.0)
            .setMaxAccelerationMetersPerSec2(4.0)
            .setMaxVelocityDegPerSec(360.0)
            .setMaxAccelerationDegPerSec2(580.0)
            .setEndTranslationToleranceMeters(0.4)
            .setEndRotationToleranceDeg(5.0);
          Path pointStart = new Path(
            constraints,
            new Path.Waypoint(new Translation2d(4.0, 7.4), new Rotation2d(-1.5707963267948966))
        );
        Path pathInt = new Path(
            constraints,
            new Path.Waypoint(new Translation2d(7.7, 7.4), new Rotation2d(-1.5707963267948966)),
            new Path.Waypoint(new Translation2d(7.7, 5.0), new Rotation2d(-1.5707963267948966)),
            new Path.Waypoint(new Translation2d(7.7, 7.4), new Rotation2d(-1.5707963267948966))
        );
        Path pointPostInt = new Path(
            constraints,
            new Path.Waypoint(new Translation2d(7.7, 7.4), new Rotation2d(-1.5707963267948966)),
            new Path.Waypoint(new Translation2d(4.0, 7.4), new Rotation2d(-1.5707963267948966))
        );
        return Commands.sequence(
            Commands.print("1"),
            basicShootInitial8(),
            Commands.print("3"),
            shooter.testSetHoodAngle(Degrees.of(0)).withTimeout(0.5),
            Commands.print("4"),
            new ParallelCommandGroup(
                pathing.followPathTeamFlipped(pathInt).withTimeout(15.0),
                Commands.sequence(
                    new WaitCommand(2.0),
                    intake.intake())),
            Commands.print("8"),
            intake.stop().withTimeout(0.2),
            Commands.print("9"),
            pathing.followPathTeamFlipped(pointPostInt).withTimeout(7.5),
            Commands.print("10"),
            basicShootInitial8()
        );
    }


    public Command notBlineCenterBlueLeftShoot(){
        return Commands.sequence(
            basicShootInitial8(),
            shooter.testSetHoodAngle(Degrees.of(0)).withTimeout(0.5),
            swerve.pidToPose(()->preIntBL).until(()->swerve.isOnTargetTranslate()),
            swerve.pidToPose(()->postIntBL).alongWith(intake.intake()).until(()->swerve.isOnTargetTranslate()),
            swerve.pidToPose(()->preIntBL).alongWith(intake.stop()).until(()->swerve.isOnTargetTranslate()),
            swerve.pidToPose(()->shotPoseBL).until(()->swerve.isOnTargetTranslate()),
            basicShootToEmpty()
        );
    }
    public Command notBlineCenterBlueRightShoot(){
        return Commands.sequence(
            basicShootInitial8(),
            shooter.testSetHoodAngle(Degrees.of(0)).withTimeout(0.5),
            swerve.pidToPose(()->preIntBR).until(()->swerve.isOnTargetTranslate()),
            swerve.pidToPose(()->postIntBR).alongWith(intake.intake()).until(()->swerve.isOnTargetTranslate()),
            swerve.pidToPose(()->preIntBR).alongWith(intake.stop()).until(()->swerve.isOnTargetTranslate()),
            swerve.pidToPose(()->shotPoseBR).until(()->swerve.isOnTargetTranslate()),
            basicShootToEmpty()
        );
    }
    public Command notBlineCenterRedLeftShoot(){
        return Commands.sequence(
            basicShootInitial8(),
            shooter.testSetHoodAngle(Degrees.of(0)).withTimeout(0.5),
            swerve.pidToPose(()->preIntRL).until(()->swerve.isOnTargetTranslate()),
            swerve.pidToPose(()->postIntRL).alongWith(intake.intake()).until(()->swerve.isOnTargetTranslate()),
            swerve.pidToPose(()->preIntRL).alongWith(intake.stop()).until(()->swerve.isOnTargetTranslate()),
            swerve.pidToPose(()->shotPoseRL).until(()->swerve.isOnTargetTranslate()),
            basicShootToEmpty()
        );
    }
    public Command notBlineCenterRedRightShoot(){
        return Commands.sequence(
            basicShootInitial8(),
            shooter.testSetHoodAngle(Degrees.of(0)).withTimeout(0.5),
            swerve.pidToPose(()->preIntRR).until(()->swerve.isOnTargetTranslate()),
            swerve.pidToPose(()->postIntRR).alongWith(intake.intake()).until(()->swerve.isOnTargetTranslate()),
            swerve.pidToPose(()->preIntRR).alongWith(intake.stop()).until(()->swerve.isOnTargetTranslate()),
            swerve.pidToPose(()->shotPoseRR).until(()->swerve.isOnTargetTranslate()),
            basicShootToEmpty()
        );
    }
    public Command depotAutoBlue(){
        return Commands.sequence(
            basicShootInitial8(),
            swerve.pidToPose(()->preIntDepotBlue).alongWith(intake.intake()).until(()->swerve.isOnTargetTranslate()),
            new ParallelCommandGroup(
                swerve.pidToPose(()->intDepotBlue),
                intake.intake(),
                shooter.shootHub(),
                spindexer.feedToShooterForce()
            )
        );
    }
    public Command depotAutoRed(){
        return Commands.sequence(
            basicShootInitial8(),
            swerve.pidToPose(()->preIntDepotRed).alongWith(intake.intake()).until(()->swerve.isOnTargetTranslate()),
            new ParallelCommandGroup(
                swerve.pidToPose(()->intDepotRed),
                intake.intake(),
                shooter.shootHub(),
                spindexer.feedToShooterForce()
            )
        );
    }
   
}