// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Questnav.QuestNavSubsystem;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Spindexer.Spindexer;
import frc.robot.Subsystems.Swerve.Pathing;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.TargetingSystem.TargetingSystem;
import frc.robot.lib.BLine.FlippingUtil;
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

        //EVIL trenchPoses, IN IS OUR STARTING TRENCH POSES
        Pose2d preTrenchInBlueLeft = new Pose2d(4.0, 7.4, new Rotation2d());
        Pose2d preTrenchInBlueRight = new Pose2d(4.0, 0.6, new Rotation2d());
        Pose2d preTrenchOutBlueRight = new Pose2d(6.0, 0.6, new Rotation2d(Degrees.of(180)));
        Pose2d preTrenchOutBlueLeft = new Pose2d(6.0, 7.4, new Rotation2d(Degrees.of(180)));
       
        //EVIL HAlFWAY POSES
        Pose2d neutralHalfWayRight = new Pose2d(8.6, 0.6, new Rotation2d(Degrees.of(120)));
        Pose2d neutralHalfwayLeft = new Pose2d();   //TODO:Flip ts

        //EVIL PRE INTAKE POSES
        Pose2d neutralPreIntRight = new Pose2d(8.6, 1.2, new Rotation2d(Degrees.of(120)));
        Pose2d neutralPreIntLeft = new Pose2d(8.6, 1.2, new Rotation2d(Degrees.of(120))); //TODO: Flip TS

        //EVIL INTAKING POSE
        Pose2d neutralIntRight = new Pose2d(8.6, 3.5, new Rotation2d(Degrees.of(120)));
        Pose2d neutralIntLeft = new Pose2d(8.6, 3.5, new Rotation2d(Degrees.of(120))); //TODO: Flip TS

        //EVIL SCOOPER THINGY
        Pose2d scoopFuel = new Pose2d(7.64, 0.88, new Rotation2d(Degrees.of(120)));
       
        //SORT OF THE BEST SHOOTING POSE
        Pose2d shotPoseBL = new Pose2d(3.6, 7.4, new Rotation2d(Degrees.of(180)));
        Pose2d shotPoseRight = new Pose2d(3.6, 0.6, new Rotation2d(Degrees.of(180)));

        //Depot poses
        Pose2d preIntDepot = new Pose2d(1.0, 6.0, new Rotation2d(Degrees.of(180)));
        Pose2d preIntDepotRed = new Pose2d(15.5, 2.0, new Rotation2d());
        Pose2d intDepot = new Pose2d(0.52, 6.0, new Rotation2d(Degrees.of(180-15)));
        Pose2d intDepotRed = new Pose2d(16.02, 2.0, new Rotation2d(Degrees.of(15)));

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
        // autoChooser.addOption("Blue Left Go Center", this::BlueLeftGoCenter);
        // autoChooser.addOption("Blue Right Go Center", this::BlueRightGoCenter);
        // autoChooser.addOption("Red Left Go Center", this::RedLeftGoCenter);
        // autoChooser.addOption("Red Right Go Center", this::RedRightGoCenter);
        autoChooser.addOption("Red Left Center Shoot Slow", this::slowTestinCenterShotAutoRedLeft);
        autoChooser.addOption("Blue Depot", this::depotAutoBlue);
        autoChooser.addOption("Red Depot", this::depotAutoRed);
        autoChooser.addOption("Evil Mentor Auto", this::evilMentorAuto);
        

    }

    //Get Auto Command
    public Command getAutonomousCommand(){
        return autoChooser.getSelected().get();
    }

    private Pose2d autoTeamFlippedPose(double x, double y, double degrees){
        var pose = new Pose2d(x,y,new Rotation2d(Degree.of(degrees)));
        if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ) pose = FlippingUtil.flipFieldPose(pose);
        return pose;
    }

    /////////////////////////
    //ALL AUTOS BELLOW HERE sk was here!!!!!!!!!//
    /////////////////////////
    public Command basicShootInitial8(){
        return Commands.sequence(
            swerve.turnToHeading(()->{
                return targeting.getHeadingToTarget(swerve.getSwervePose().getTranslation(), targeting.getHubTarget()).plus(Rotation2d.k180deg);
            }).until(()->swerve.isOnTargetAngle()).withTimeout(1.0),
            shootAuto().withTimeout(1.5)
        );
    }
    public Command basicShootToEmpty(){
        return Commands.sequence(
            swerve.turnToHeading(()->{
                return targeting.getHeadingToTarget(swerve.getSwervePose().getTranslation(), targeting.getHubTarget()).plus(Rotation2d.k180deg);
            }).until(()->swerve.isOnTargetAngle()).withTimeout(1.0),
            shootAuto().withTimeout(3)
        );
    }

    public Command depotAutoBlue(){
        return Commands.sequence(
            basicShootInitial8(),
            swerve.pidToPose(()->preIntDepot).alongWith(intake.intake()).until(()->swerve.isOnTargetTranslate()),
            driveToPose(intDepot, intakeWhileShooting()),
            intakeWhileShooting()
            );
    }

    public Command depotAutoRed(){
        return Commands.sequence(
            basicShootInitial8(),
            driveToPose(preIntDepot, stow()),
            driveToPose(intDepot, intakeWhileShooting()),
            intakeWhileShooting()
        );
    }

    public Command slowTestinCenterShotAutoRedLeft(){
        Pose2d first = new Pose2d(9.25, 0.6, new Rotation2d());
        Pose2d second = new Pose2d(9.25, 3.0, new Rotation2d(Degrees.of(135)));
        Pose2d third = new Pose2d(14, 0.6, new Rotation2d());
        return Commands.sequence(
            swerve.pidToPose(()->first).until(()->swerve.isOnTargetTranslate()),
            swerve.pidToPose(()->second, 1.0, Inches.of(5)).alongWith(intake.intake()).until(()->swerve.isOnTargetTranslate()),
            swerve.pidToPose(()->first).until(()->swerve.isOnTargetTranslate()),
            swerve.pidToPose(()->third).until(()->swerve.isOnTargetTranslate()),
            shootAuto().withTimeout(10)
        );
    }

    public Command evilMentorAuto(){
        return Commands.sequence(
            driveToPose(preTrenchInBlueRight, stow()), //near trench
            driveToPose(preTrenchOutBlueRight, stow()), //far trench
            //load up in the middle across center line
            driveToPose(neutralHalfWayRight, intakeWhilePassing()),
            driveToPose(neutralPreIntRight, intakeWhilePassing()),
            driveToPoseSlowly(neutralIntRight, intakeWhilePassing()),
            //zoom back, pulling any fuel toward the trench
            driveToPose(scoopFuel, intakeWhilePassing()),
            //Drive to and through trench
            driveToPose(preTrenchOutBlueRight, intakeWhilePassing()),
            driveToPose(shotPoseRight, intakeOnly()),
            //drive along the wall, satisfied with a job well done
            driveToPoseSlowly(0.5, 0.5, 180, intakeWhileShooting()),
            Commands.none()
        )
        .withTimeout(20)
        ;
    }

    public Command evilAuto(){
        return Commands.sequence(


        ).withTimeout(21);
    }


    //////////////////////////////////////////
    /// SuperStructure State Commands ///////
    ////////////////////////////////////////
    
    public Command driveToPose(double x, double y, double degrees, Command superState){
        return driveToPose(new Pose2d(x, y, new Rotation2d(Degrees.of(degrees))), superState);
    }

    public Command driveToPoseSlowly(double x, double y, double degrees, Command superState){
        return driveToPoseSlowly(new Pose2d(x, y, new Rotation2d(Degrees.of(degrees))), superState);
    }
    
    public Command driveToPose(Pose2d targetPose, Command superState){
        return swerve.pidToPose(()->autoTeamFlippedPose(targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees()))
        .alongWith(superState)
        .until(()->swerve.isOnTargetTranslate())
        ;
    }

    public Command driveToPoseSlowly(Pose2d targetPose, Command superState){
        return swerve.pidToPose(()->autoTeamFlippedPose(targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees()), 0.5, Inches.of(5))
        .alongWith(superState)
        .until(()->swerve.isOnTargetTranslate())
        ;
    }

    public Command stow(){
        return Commands.parallel(
            shooter.stow(),
            spindexer.stop(),
            intake.stop()
        );
    }

    public Command pass(){
        return new ParallelCommandGroup(
            shooter.pass(),
            spindexer.feedToShooterForce(),
            intake.stop()
        );
    }

    public Command shootAuto(){
        return new ParallelCommandGroup(
            shooter.shootHubNoTur(),
            spindexer.feedToShooterForce(),
            intake.stop()
        );
    }

    public Command intakeOnly(){
        return new ParallelCommandGroup(
            shooter.stow(),
            spindexer.stop(),
            intake.intake()
        );
    }

    public Command intakeWhilePassing(){
        return new ParallelCommandGroup(
            shooter.pass(),
            spindexer.feedToShooterForce(),
            intake.intake()
        );
    }

    public Command intakeWhileShooting(){
        return new ParallelCommandGroup(
            shooter.shootHubVelComp(),
            spindexer.feedToShooterForce(),
            intake.intake()
        );
    }    
}