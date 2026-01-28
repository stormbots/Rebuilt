// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Questnav;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;
import frc.robot.Subsystems.Photonvision.Photonvision;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;



public class QuestNavSubsystem extends SubsystemBase {
  /** Creates a new QuestNavSubsystem. */
  QuestNav questNav = new QuestNav();
  Field2d questField2d = new Field2d();
  public Field2d occulusField = new Field2d();
  SwerveSubsystem swerve;
  Matrix<N3, N1> QUESTNAV_STD_DEVS =
    VecBuilder.fill(
        0.20, // Trust down to 2cm in X direction
        0.20, // Trust down to 2cm in Y direction
        0.35 // Trust down to 2 degrees rotational
    );

  Transform3d robotToQuest = new Transform3d(new Translation3d(Inches.of(7).in(Meters), Inches.of(-6).in(Meters), Inches.of(8.0).in(Meters)), new Rotation3d(0.0, 0.0, Units.degreesToRadians(-135)));

  public QuestNavSubsystem(SwerveSubsystem swerve) {
    SmartDashboard.putData("questnav/occulusField", occulusField);
    this.swerve = swerve;
    setInitialPose(new Pose3d());
  }

  @Override
  public void periodic() {
    questNav.commandPeriodic();
    // Get the latest pose data frames from the Quest
    PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

    SmartDashboard.putNumber("questnav/numFrames", questFrames.length);

    // Loop over the pose data frames and send them to the pose estimator
    for (PoseFrame questFrame : questFrames) {
        // Make sure the Quest was tracking the pose for this frame
        // if (questFrame.isTracking()) {
            SmartDashboard.putBoolean("questnav/tracking", true);
            // Get the pose of the Quest
            Pose3d questPose = questFrame.questPose3d();
            // Get timestamp for when the data was sent
            double timestamp = questFrame.dataTimestamp();

            // Transform by the mount pose to get your robot pose
            Pose3d robotPose = questPose.transformBy(robotToQuest.inverse());

            // You can put some sort of filtering here if you would like!

            // Add the measurement to our estimator
            occulusField.setRobotPose(robotPose.toPose2d());
            
            //swerve.swerveDrive.addVisionMeasurement(robotPose.toPose2d(), timestamp, QUESTNAV_STD_DEVS);
        // }
        // else{
          // SmartDashboard.putBoolean("questnav/tracking", false);
        // }  
    }

    SmartDashboard.putBoolean(" questnav/connected", questNav.isConnected());
  }

  public void setInitialPose(Pose3d startPose){
    Pose3d questStartPose = startPose.transformBy(robotToQuest);
    questNav.setPose(questStartPose);
  }
}
