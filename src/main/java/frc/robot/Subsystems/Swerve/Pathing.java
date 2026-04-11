// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Swerve;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Spindexer.Spindexer;
import frc.robot.Subsystems.TargetingSystem.TargetingSystem;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;


public class Pathing extends SubsystemBase {
  /** Creates a new Pathing. */
  Swerve swerve;
  Shooter shooter;
  Intake intake;
  Spindexer spindexer;
  TargetingSystem targeting;

  FollowPath.Builder pathBuilder;

  public Pathing(Swerve swerve,
        Shooter shooter,
        Intake intake,
        Spindexer spindexer,
        TargetingSystem targeting) {
    this.swerve = swerve;
    this.shooter = shooter;
    this.intake = intake;
    this.targeting = targeting;
    this.spindexer = spindexer;
    pathBuilder = new FollowPath.Builder(
    swerve, 
    swerve::getSwervePose, 
    swerve::getChassisSpeedsRobotRelative, 
    this::setAutoInputs, 
    new PIDController(2.5, 0.0, 0.0),    // Translation PID
    new PIDController(1.0, 0.0, 0.0),    // Rotation PID
    new PIDController(0.0, 0.0, 0.0)     // Cross-track PID
    ).withTRatioBasedTranslationHandoffs(true)
    ;
  
    FollowPath.registerEventTrigger("intake", intake.intake());
    FollowPath.registerEventTrigger("shoot", shootAuto());
    FollowPath.registerEventTrigger("intakeWhileShooting", intakeWhileShooting());
    FollowPath.registerEventTrigger("intakeWhilePassing", intakeWhilePassing());
    FollowPath.registerEventTrigger("pass", pass());
    FollowPath.registerEventTrigger("intakeStop", intake.stop().asProxy());
    FollowPath.registerEventTrigger("intakeStow", intake.stow().asProxy());
    FollowPath.registerEventTrigger("hoodDown", shooter.testSetHoodAngle(Degrees.of(0)));
    FollowPath.registerEventTrigger("stopShooting", stopShooting());
  }

  public Command followPath(Path path){
    return pathBuilder.build(path);
  }

  public Command followPathTeamFlipped(Path path){
    path.flip();
    return pathBuilder.build(path);
  }

  private void setAutoInputs(ChassisSpeeds robotRelative) {
    //Converting robot relative from bline for field relative inputs
    Rotation2d heading = swerve.getSwervePose().getRotation(); 
    ChassisSpeeds fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelative, heading);
    swerve.setPrimaryInputsVoid(()->fieldRelative.vxMetersPerSecond, ()->fieldRelative.vyMetersPerSecond, ()->fieldRelative.omegaRadiansPerSecond);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command stopShooting(){
    return new ParallelCommandGroup(
      shooter.testSetHoodAngle(Degrees.of(0)),
      spindexer.stop()
    );
  }

  public Command pass(){
    return new ParallelCommandGroup(
      shooter.pass(),
      spindexer.feedToShooterForce()
    );
  }

  public Command shootAuto(){
    return new ParallelCommandGroup(
      shooter.shootHubVelComp(),
      spindexer.feedToShooterForce()
    );
  }

  public Command intakeWhilePassing(){
    return new ParallelCommandGroup(
      intake.intake(),
      shooter.pass(),
      spindexer.feedToShooterForce()
    );
  }

  public Command intakeWhileShooting(){
    return new ParallelCommandGroup(
      intake.intake(),
      shooter.shootHubVelComp(),
      spindexer.feedToShooterForce()
    );
  }
}
