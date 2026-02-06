// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Questnav;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Swerve.Swerve;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;


public class QuestNavSubsystem extends SubsystemBase {
  /** Creates a new QuestNav. */
  Swerve swerveSubsystem;
  public QuestNavSubsystem(Swerve swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
  }

  //Values to change when we get bot
  Transform3d robotToQuest = new Transform3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));

  QuestNav questNav = new QuestNav();
  Matrix<N3, N1> QUESTNAV_STD_DEVS =
      VecBuilder.fill(
        0.02, // Trust down to 2cm in X direction
          0.02, // Trust down to 2cm in Y direction
      0.035 // Trust down to 2 degrees rotational
      );

@Override
  public void periodic() {
    // Get the latest pose data frames from the Quest
    PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();
    // Loop over the pose data frames and send them to the pose estimator
    for (PoseFrame questFrame : questFrames) {
        // Make sure the Quest was tracking the pose for this frame
        SmartDashboard.putBoolean("Questnav/isTracking", questFrame.isTracking());
        if (questFrame.isTracking()) {
            // Get the pose of the Quest
            Pose3d questPose = questFrame.questPose3d();
            // Get timestamp for when the data was sent
            double timestamp = questFrame.dataTimestamp();
            // Transform by the mount pose to get your robot pose
            Pose3d robotPose = questPose.transformBy(robotToQuest.inverse());
            //add to swervedrive pose
            swerveSubsystem.addVisionMeasurement(robotPose.toPose2d(), timestamp, QUESTNAV_STD_DEVS);
        }
    }
  }

  public void setQuestPose(Pose3d robotPose)
  {
    questNav.setPose(robotPose.transformBy(robotToQuest));
  }
}
