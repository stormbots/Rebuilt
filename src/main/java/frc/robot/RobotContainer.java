// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;


import static edu.wpi.first.units.Units.Degrees;

import com.stormbots.CRTAbsoluteEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.FieldBehaviour;
import frc.robot.Subsystems.Climber.Climber;
import frc.robot.Subsystems.HopperSensors.HopperSensors;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Lighting.Signals;
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
  Autos autos = new Autos(swerve, shooter, intake, questnav, spindexer, pathing, targeting);
  ShiftTracking shiftTracking = new ShiftTracking();
  // Bling bling = new Bling(); //TODO: Currently no bling lights on bot
  FieldBehaviour fieldBehaviour = new FieldBehaviour();




  CommandXboxController driver = new CommandXboxController(0);
  CommandXboxController operator = new CommandXboxController(1);
  CommandXboxController debug = new CommandXboxController(3);
  Path testingPath = new Path("goCollect");
  Double rpm = 2600.0;
  Double hoodAngle = 25.0;


  public void syncQuestPose (){
    questnav.setQuestPose(new Pose3d(swerve.getSwervePose()));
  };


  public RobotContainer() {
    SmartDashboard.putNumber("robotContainer/flywheelrpm", rpm);
    SmartDashboard.putNumber("robotContainer/hoodAngle", hoodAngle);


    HopperSensors.getInstance(); //Ensure this always exists and is updating
    questnav.setQuestPose(new Pose3d(swerve.swerveDrive.getPose().getX(), swerve.swerveDrive.getPose().getY(), 0.0, new Rotation3d(0.0, 0.0, 0.0)));
   
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




    new Trigger(photonvision::doesNotHaveTarget).and(DriverStation::isDisabled).and(()->DriverStation.getAlliance().isPresent())
    .onFalse(wled.signals.showVisionOkay());

    ShiftTracking.canShoot.onTrue(wled.signals.shiftStart()).onFalse(wled.signals.shiftEnd());

    new Trigger(DriverStation::isTeleopEnabled).onTrue(wled.signals.Cancel());

    new Trigger(DriverStation::isAutonomousEnabled).onTrue(wled.signals.Cancel());



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
  }


  private void configureDriverBindings() {
    swerve.setDefaultCommand(swerve.setPrimaryInputs(
      ()->-driver.getLeftY()*2.0,
      ()->-driver.getLeftX()*2.0,
      ()->-driver.getRightX()*2.0
    ));




    //TODO: PROGRAMMING DEBUG BUTTONS CODE REMOVE ME


    driver.start().onTrue(
      Commands.sequence(
        new InstantCommand(()->questnav.wantToTrack(false)),
        swerve.zeroGyro(),
        //    IFFFF QUEST IS GOOD BUT CAMS AREN'T, UNCOMMENT THIS AND HAVE JACOB GO TO CORNER FOR ZERO
        // new InstantCommand(()->questnav.setQuestPose(new Pose3d(swerve.getSwervePose().getX(), swerve.getSwervePose().getY(), 0.0, new Rotation3d(0.0, 0.0, 0.0))))
        Commands.none()
      ).withTimeout(0.1)
    );
     


    driver.rightTrigger()
      .whileTrue(swerve.turnToHeading(()->new Rotation2d(Degrees.of(-90)))
    );
    driver.leftTrigger()
      .whileTrue(swerve.turnToHeading(()->new Rotation2d(Degrees.of(90)))
    );
   
    // END PROGRAMMING DEBUG BUTTONS

    driver.leftBumper().whileTrue(
      swerve.setPrimaryInputs(
      ()->-driver.getLeftY()/3.0,
      ()->-driver.getLeftX()/3.0,
      ()->-driver.getRightX()/3.0
      )
    );

    driver.a().whileTrue(swerve.turnToHeading(()->new Rotation2d())); // face swerve away from driver
    // driver.b() //TODO: face any 45 to prepare for bump crossing

    driver.x() // extend intake // This button is useless and will never be used
    .whileTrue(intake.intake())

    .whileFalse(intake.stop());

    driver.y().whileTrue(intake.eject()); //intake eject //also will never be used

    // driver.(back left paddle) //global stow/defense mode

    driver.povUp().whileTrue(shooter.testHome());

  }


  private void configureOperatorBindings() {

    operator.rightTrigger()
    .whileTrue(shootHub());

    operator.rightBumper()
    .whileTrue(fixedShot());

    operator.leftTrigger()
    .whileTrue(pass());

    operator.leftBumper()
    .whileTrue(fixedPass());
    //DO TS LATER
    // operator.povDown()
    // .whileTrue(globalStow());

    operator.povUp()
    .whileTrue(spindexer.unclog());

    operator.b()
    .whileTrue(climber.prepareForClimbL1())
    .onFalse(climber.climbL1());
    //MAN GRABBER DO TS
    // operator.y()
    // .whileTrue(grabberstuffs);
    // STAGE 2 STUFFS
    // operator.x()
    // .whileTrue(command);
    // operator.leftBumper()
    //   .whileTrue(climber.prepareForClimbL1())
    //   .onFalse(climber.climbL1())
    // ;

    // operator.rightBumper()
    // .whileTrue(fixedPassOpp());
    // operator.y()
    // .whileTrue(fixedPass())
    // ;

    // //  CLIMBER STUFFS, OBVIOUSLY MASSIVE COMMENTED CODE IS CHOPPED BUT NEEDS TO STAY FOR NOW
    // // operator.rightBumper() stage2 hook up, lock out if not end of match  
    // // operator.rightBumper().whileTrue(climber.setStage2Voltage(12));
    // // // operator.rightTrigger() stage2 hook down, lock out if not end of match
    // // operator.rightTrigger().whileTrue(climber.setStage2Voltage(-12));
    // operator.povUp().whileTrue(spindexer.unclog()); // shake dye rotor / unclog

    // operator.povDown().whileTrue(intake.eject()); // intake.eject()

    // operator.x() // shoot + hopper feed
    // .whileTrue(shootHub())
    // ;

    // //TALK TO ABBY MAKE THIS A DIFFERENT BUTTON
    // operator.povRight()
    // .whileTrue(fixedShot());

    // operator.a().whileTrue(climber.stow()); // global stow (unnecessary, this is default)
    // operator.povLeft().whileTrue(climber.goHome());


    // operator.b()// passing: Face driver station wall and launch at fixed rpm/angle/distance
    // .whileTrue(pass())
    // ;

    // // operator.back() // re-home intake, hood, turret? hood? not doing this rn

  }

  //BUTTON FUNCTIONS/STATES
  public Command pass(){
    return new ParallelCommandGroup(
      swerve.verifyAngleTargetPass(()->{
        return targeting.getHeadingToTarget(swerve.getSwervePose().getTranslation(), targeting.getPassTarget()).plus(Rotation2d.k180deg);
      }),
      shooter.pass(),
      spindexer.feedToShooter()
    );
    }
    public Command shootHub(){
      return new ParallelCommandGroup(
        swerve.turnToHeadingWithinTurretRange(()->{
          return targeting.getHeadingToTarget(swerve.getSwervePose().getTranslation(), targeting.getHubTarget()).plus(Rotation2d.k180deg);
        }),
        shooter.shootHub(),
        spindexer.feedToShooter()
        );
    }
    public Command fixedShot(){
      return new ParallelCommandGroup(
        shooter.shoot(()->targeting.fixedShot()),
        spindexer.feedToShooter()
      );
    }
    public Command fixedPass(){
      return new ParallelCommandGroup(
        shooter.shoot(()->targeting.fixedPassNeutral()),
        spindexer.feedToShooter()
      );
    }
    public Command fixedPassOpp(){
      return new ParallelCommandGroup(
        shooter.shoot(()->targeting.fixedPassOppAlliance()),
        spindexer.feedToShooter()
      );
    }

  public Command getAutonomousCommand() {
    //TODO: Get this from Autos.java instead
   
    return Commands.print("No autonomous command configured");
  }
}
