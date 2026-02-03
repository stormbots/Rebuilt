// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Photonvision;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meters;

import java.util.List;
import java.util.Optional;
import java.util.Vector;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Swerve.Swerve;

public class Photonvision extends SubsystemBase {
  Swerve swerve;
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
  NetworkTableInstance table = NetworkTableInstance.getDefault();
  Optional<PhotonCamera> centerCamera;
  Matrix<N3, N1> currentStdDevs;
  Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(4, 4, 8);
  Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  Transform3d cameraToCenter = new Transform3d(new Translation3d(
    Inch.of(6).in(Meters), 
    Inch.of(6).in(Meters), 
    Inch.of(0).in(Meters)), 
    new Rotation3d(0.0, 0.0, 45.0)
    );

    Field2d visionField2d = new Field2d();

    PhotonPoseEstimator centerEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, cameraToCenter);
  /** Creates a new Photonvision.
   *  @param swerve */
  public Photonvision(Swerve swerve) {
    this.swerve = swerve;
    SmartDashboard.putData("visionfield", visionField2d);

    try{
      centerCamera = Optional.of(new PhotonCamera("Arducam_OV9782_USB_Camera"));
    }
    catch(Error e){
      System.err.print(e);
      centerCamera = Optional.empty();
    }
  }

  public void updateOdometry(){
    if(centerCamera.isPresent()){
      updateCameraSideOdometry(centerEstimator, centerCamera.get());
    }
  }

  private void updateCameraSideOdometry(PhotonPoseEstimator poseEstimator, PhotonCamera camera){
    Optional<EstimatedRobotPose> visionEstimate = Optional.empty();
    for(var result : camera.getAllUnreadResults()){
      visionEstimate = poseEstimator.estimateCoprocMultiTagPose(result);
      if (visionEstimate.isEmpty()){
        visionEstimate = poseEstimator.estimateLowestAmbiguityPose(result);
      }
      updateEstimationStdDevs(visionEstimate, result.getTargets());
    }

    visionEstimate.ifPresent(
      est ->{
        var estimatedStdDevs = getEstimationStdDevs();

        swerve.swerveDrive.addVisionMeasurement(est.estimatedPose.toPose2d(),est.timestampSeconds, estimatedStdDevs);
        visionField2d.getObject(camera.getName()).setPose(est.estimatedPose.toPose2d());
      }
      
    );
  }

  public void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets){
    if (estimatedPose.isEmpty()){
      currentStdDevs = singleTagStdDevs;
    }
    else{
      var estimatedStdDevs = singleTagStdDevs;
      int numTags = 0;
      double avgDistance = 0.0;

      for(var tag : targets){
        var tagPose = centerEstimator.getFieldTags().getTagPose(tag.getFiducialId());
        if (tagPose.isEmpty()) continue;
        numTags++;
        avgDistance += tagPose
          .get()
          .toPose2d()
          .getTranslation()
          .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0){
        currentStdDevs = singleTagStdDevs;
      }
      else{
        avgDistance /= numTags;

        if (numTags<1) estimatedStdDevs = multiTagStdDevs;

        if (numTags == 1 && avgDistance >4){
          estimatedStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        }
        else estimatedStdDevs = estimatedStdDevs.times(1+(avgDistance*avgDistance/30));
        currentStdDevs = estimatedStdDevs;
      }
    }
  }

  public Matrix<N3, N1> getEstimationStdDevs() {
        return currentStdDevs;
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    visionField2d.setRobotPose(swerve.getSwervePose());
    updateOdometry();
  }
}
