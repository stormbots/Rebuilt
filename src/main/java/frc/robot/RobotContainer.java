// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;


import static edu.wpi.first.units.Units.Degrees;

import com.stormbots.CRTAbsoluteEncoder;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.FieldBehaviour;
import frc.robot.Subsystems.Climber.Climber;
import frc.robot.Subsystems.HopperSensors.HopperSensors;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Lighting.WLED;
import frc.robot.Subsystems.Photonvision.Photonvision;
import frc.robot.Subsystems.Questnav.QuestNavSubsystem;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Spindexer.Spindexer;
import frc.robot.Subsystems.Swerve.Pathing;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.TargetingSystem.TargetingSystem;
import frc.robot.lib.BLine.Path;


public class RobotContainer {


  Swerve swerve = new Swerve();
  Photonvision photonvision = new Photonvision(swerve);
  TargetingSystem targeting = new TargetingSystem(swerve);
  Shooter shooter = new Shooter(targeting);
  Intake intake = new Intake();
  Climber climber = new Climber();
  Spindexer spindexer = new Spindexer(shooter.isReadyToAcceptFuel.and(swerve::isOnTargetAngle));
  QuestNavSubsystem questnav = new QuestNavSubsystem(swerve);
  Pathing pathing = new Pathing(swerve, shooter, intake, spindexer, targeting);
  WLED wled = new WLED(new SerialPort(115200, Port.kUSB1),photonvision);
  Autos autos = new Autos(swerve, shooter, intake, questnav, spindexer, pathing, targeting, climber);
  ShiftTracking shiftTracking = new ShiftTracking();
  // Bling bling = new Bling(); //TODO: Currently no bling lights on bot
  FieldBehaviour fieldBehaviour = new FieldBehaviour();




  CommandXboxController driver = new CommandXboxController(0);
  CommandXboxController operator = new CommandXboxController(1);
  // CommandXboxController debug = new CommandXboxController(3);
  Path testingPath = new Path("goCollect");
  Double rpm = 2600.0;
  Double hoodAngle = 25.0;


  public void syncQuestPose (){
    questnav.setQuestPose(new Pose3d(swerve.getSwervePose()));
  };

  public Command questWantToTrack(){
    return questnav.wantToTrackCommand(true);
  }


  public RobotContainer() {
    SmartDashboard.putNumber("robotContainer/flywheelrpm", rpm);
    SmartDashboard.putNumber("robotContainer/hoodAngle", hoodAngle);


    HopperSensors.getInstance(); //Ensure this always exists and is updating
   
    configureDriverBindings();
    configureOperatorBindings();
    configureDebugBindings();


   
    CRTAbsoluteEncoder.getInstance().sync();

    //I DONT WANT TO GET RID OF THIS STUFF YET IDK WHATS GOING ON HERE
    // new Trigger(DriverStation::isEnabled)
    // .whileTrue(
    //   swerve.addFieldInput( ()->fieldBehaviour.getSwerveInputs(swerve.getSwervePose()) )
    // );


    // syncQuestPose.runsWhenDisabled();


    // new Trigger(()->Timer.getFPGATimestamp() > 1)
    // .and(DriverStation::isDisabled)
    // .whileTrue(syncQuestPose);




    new Trigger(photonvision::doesNotHaveTarget)
    .and(DriverStation::isDisabled)
    .and(()->DriverStation.getAlliance().isPresent())
    .onFalse(wled.signals.showVisionOkay())
    .onTrue(wled.signals.reboot());

    ShiftTracking.canShoot.onTrue(wled.signals.shiftStart()).onFalse(wled.signals.shiftEnd());

    new Trigger(DriverStation::isEnabled).onTrue(wled.signals.reboot());

    new Trigger(DriverStation::isEnabled).and(DriverStation::isFMSAttached).onTrue(WLED.setAuraMode());



    //while disabled
    //and see target
    //update quest pose
    //onEnable


  }


  private void configureDebugBindings(){
    // debug.a().whileTrue(swerve.pidToPose(()->new Pose2d(4.0, 0.6, new Rotation2d())));
    // debug.b().whileTrue(swerve.pidToPose(()->new Pose2d(7.7, 0.6, new Rotation2d(-Math.PI/2))));
    // debug.x().whileTrue(swerve.pidToPose(()->new Pose2d(4.0, 7.4, new Rotation2d())));
    // debug.y().whileTrue(swerve.pidToPose(()->new Pose2d(7.7, 7.4, new Rotation2d(Math.PI/2))));
    // debug.povRight().whileTrue(swerve.pidToPose(()->new Pose2d(12.5, 7.4, new Rotation2d())));
    // debug.povLeft().whileTrue(swerve.pidToPose(()->new Pose2d(12.5, 0.6, new Rotation2d())));
    // debug.x()
    // .whileTrue(shooter.shootWithDashboardValues())
    // .whileTrue(Commands.waitSeconds(1).andThen(spindexer.feedToShooter()))
    // ;

    // debug.a()
    // .whileTrue(shooter.testTurretVoltage(()->debug.getLeftY()*8));
  }


  private void configureDriverBindings() {
    swerve.setDefaultCommand(swerve.setPrimaryInputs(
      ()->-driver.getLeftY()*2.0,
      ()->-driver.getLeftX()*2.0,
      ()->-driver.getRightX()*2.0
    ));

    driver.start().onTrue(
      Commands.sequence(
      questnav.wantToTrackCommand(false),
      swerve.zeroGyro()
      )
    );

    driver.povLeft().onTrue(
      Commands.sequence(
        questnav.setQuestPoseCommand(
          ()->swerve.getSwervePose()
        ),
        new WaitCommand(0.3),
        questnav.wantToTrackCommand(true)
      )
    );
     
    driver.rightTrigger()
      .whileTrue(swerve.turnToHeading(()->new Rotation2d(Degrees.of(-90)))
    );
    driver.leftTrigger()
      .whileTrue(swerve.turnToHeading(()->new Rotation2d(Degrees.of(90)))
    );

    driver.leftBumper().whileTrue(
      swerve.setPrimaryInputs(
      ()->-driver.getLeftY()/3.0,
      ()->-driver.getLeftX()/3.0,
      ()->-driver.getRightX()/3.0
      )
    );

    driver.a().whileTrue(swerve.turnToHeading(()->new Rotation2d())); // face swerve away from driver

    driver.x() // extend intake
    .whileTrue(intake.intake())
    .whileFalse(intake.stop());

    driver.y().whileTrue(intake.eject()); //intake eject

    driver.povUp().whileTrue(shooter.testHome());
  }


  private void configureOperatorBindings() {

    operator.rightTrigger()
    .whileTrue(shootHub())
    .whileTrue(wled.signals.automaticShot().repeatedly());
    ;

    operator.rightBumper()
    .whileTrue(fixedShot())
    .whileTrue(wled.signals.manualShot().repeatedly());
    ;

    operator.povLeft()
    .whileTrue(shooter.shootWithDashboardValues())
    .whileTrue(Commands.waitSeconds(1).andThen(spindexer.feedToShooter()))
    ;

    operator.leftTrigger()
    .whileTrue(pass())
    .whileTrue(wled.signals.automaticShot().repeatedly());
    ;

    //Simple Climber Lineup
    operator.leftBumper()
    .whileTrue(fixedPass())
    .whileTrue(wled.signals.manualShot().repeatedly());
    ;

    operator.povUp()
    .whileTrue(spindexer.unclog());

    operator.povDown()
    .whileTrue(climber.stow())
    .whileTrue(intake.stow());

    operator.b()
    .whileTrue(climber.prepareForClimbL1())
    .onFalse(climber.climbL1());

    operator.x()
    .whileTrue(shooter.shootWithDashboardValues())
    .whileTrue(Commands.waitSeconds(1).andThen(spindexer.feedToShooterForce()))
    .whileTrue(wled.signals.wrongShot().repeatedly())
    ;

    operator.a()
    .whileTrue(fixedPassOpp())
    .whileTrue(wled.signals.manualShot().repeatedly());
    ;

    //Operator Climber Lineup command
    //NOTE: Driver may want a swerve.turnToHeading() for this; 
    //Omitted due to prior odometry issues proving to be a risk factor
    operator.y()
    .whileTrue(Commands.parallel(
      // climber.prepareForClimbL1(),
      swerve.addSecondaryInputsTrueFielcentric(()->climber.generateSwerveInputs(swerve.getSwervePose())),

      Commands.none()
    )
    // .until(climber::isLinedUpWithL1)
    // .andThen(climber.climbL1())
    );

    operator.povRight()
    .whileTrue(shootHub())
    ;

    // //TALK TO ABBY MAKE THIS A DIFFERENT BUTTON
    // operator.povRight()
    // .whileTrue(fixedShot());
  }

  //BUTTON FUNCTIONS/STATES
  public Command pass(){
    return new ParallelCommandGroup(
      swerve.verifyAngleTargetPass(()->{
        return targeting.getHeadingToTarget(swerve.getSwervePose().getTranslation(), targeting.getPassTarget()).plus(new Rotation2d(Degrees.of(-153.5)));
      }),
      shooter.pass(),
      spindexer.feedToShooter()
    );
  }

  public Command shootHub(){
    return new ParallelCommandGroup(
      swerve.turnToHeadingWithinTurretRange(()->{
        return targeting.getHeadingToTarget(swerve.getSwervePose().getTranslation(), targeting.getHubTarget()).plus(new Rotation2d(Degrees.of(-153.5)));
      }),
      shooter.shootHubVelComp(),
      spindexer.feedToShooter()
    );
  }

  public Command fixedShot(){
    return new ParallelCommandGroup(
      shooter.shoot(()->targeting.fixedShot()),
      new WaitCommand(0.5).andThen(spindexer.feedToShooterForce())
    );
  }

  public Command fixedPass(){
    return new ParallelCommandGroup(
      shooter.shoot(()->targeting.fixedPassNeutral()),
      new WaitCommand(0.5).andThen(spindexer.feedToShooterForce())
    );
  }

  public Command fixedPassOpp(){
    return new ParallelCommandGroup(
      shooter.shoot(()->targeting.fixedPassOppAlliance()),
      new WaitCommand(0.5).andThen(spindexer.feedToShooterForce())
    );
  }
}
