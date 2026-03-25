// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Questnav;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Swerve.Swerve;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;


public class QuestNavSubsystem extends SubsystemBase {
  /** Creates a new QuestNav. */
  Field2d field = new Field2d();
  Swerve swerveSubsystem;
  private boolean wantToTrack = true;
  public QuestNavSubsystem(Swerve swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    SmartDashboard.putData("QuestField", field);
  }

  //Values to change when we get bot
  Transform3d robotToQuest = new Transform3d(
    Inches.of((-27.5/2.0 + 4.5)).in(Meters), 
    Inches.of(-27.5/2 + 2.5).in(Meters), 
    Inches.of(17.75).in(Meters), 
    new Rotation3d(
      Degrees.of(-90).in(Radians), 
      0.0, 
      Degrees.of(180).in(Radians)
    )
  );

  QuestNav questNav = new QuestNav();
  Matrix<N3, N1> QUESTNAV_STD_DEVS =
      VecBuilder.fill(
        0.03, // Trust down to 2cm in X direction
          0.03, // Trust down to 2cm in Y direction
      0.035*2.5 // Trust down to 5 degrees rotational, .035 is 2 deg,
      );

@Override
  public void periodic() {
    questNav.commandPeriodic();
    SmartDashboard.putBoolean("Questnav/isconnected", questNav.isConnected());
    SmartDashboard.putBoolean("Questnav/wantToTrack", wantToTrack);
    // Get the latest pose data frames from the Quest
    PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();
    SmartDashboard.putNumber("Questnav/frames", questFrames.length);
    // Loop over the pose data frames and send them to the pose estimator
    for (PoseFrame questFrame : questFrames) {
        // Make sure the Quest was tracking the pose for this frame
        // if (questFrame.isTracking()) {
        //if not isEnabled check, then there is no way for the cameras to start feeding into the quest
        if(questNav.isConnected()&&wantToTrack&&DriverStation.isEnabled()){
            // Get the pose of the Quest
            Pose3d questPose = questFrame.questPose3d();
            // Get timestamp for when the data was sent
            double timestamp = questFrame.dataTimestamp();
            // Transform by the mount pose to get your robot pose
            Pose3d robotPose = questPose.transformBy(robotToQuest.inverse());
            //add to swervedrive pose
            field.setRobotPose(robotPose.toPose2d());
            swerveSubsystem.addVisionMeasurement(robotPose.toPose2d(), timestamp, QUESTNAV_STD_DEVS);
        }
    }
  }

  public void setQuestPose(Pose3d robotPose)
  {
    questNav.setPose(robotPose.transformBy(robotToQuest));
  }

  public Command setQuestPoseCommand()
  {
    return run(()->setQuestPose(new Pose3d(new Pose2d(4.0, 5.88, new Rotation2d()))));
  }

  public Command setQuestPoseCommand(Pose3d robotPose)
  {
    return run(()->setQuestPose(robotPose));
  }

  public Command wantToTrackCommand(boolean bool)
  {
    return run(()->wantToTrack(bool));
  }


  //This is a very goofy way to fix how occulus stores its pose, might do this differently later but it works for now
  public void wantToTrack(boolean wantTo){
    wantToTrack = wantTo;
  }
}
