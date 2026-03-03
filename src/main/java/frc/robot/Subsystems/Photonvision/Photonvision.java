// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Photonvision;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meters;

import java.util.List;
import java.util.Optional;

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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Swerve.Swerve;

public class Photonvision extends SubsystemBase {
  private Swerve swerve;

  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  private Optional<PhotonCamera> rightCamera = Optional.empty();
  private Optional<PhotonCamera> leftCamera = Optional.empty();

  private Matrix<N3, N1> currentStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
  private Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(4, 4, 8);
  private Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  private Transform3d rightCameraToCenter = new Transform3d(new Translation3d(
    Inch.of(12).in(Meters), 
    Inch.of(-9).in(Meters), 
    Inch.of(18.65).in(Meters)), 
    new Rotation3d(0.0, 0.0, Math.toRadians(-56.5))
  );
  private Transform3d leftCameraToCenter = new Transform3d(new Translation3d(
    Inch.of(12).in(Meters), 
    Inch.of(9).in(Meters), 
    Inch.of(18.65).in(Meters)), 
    new Rotation3d(0.0, 0.0, Math.toRadians(56.5))
  );

  private Field2d visionField2d = new Field2d();
  private PhotonPoseEstimator rightEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, rightCameraToCenter);
  private PhotonPoseEstimator leftEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, leftCameraToCenter);

  /** Creates a new Photonvision.
   *  @param swerve 
   */
  public Photonvision(Swerve swerve) {
    this.swerve = swerve;
    SmartDashboard.putData("visionfield", visionField2d);

    try{
      rightCamera = Optional.of(new PhotonCamera("Right"));
      leftCamera = Optional.of(new PhotonCamera("Left"));
    }
    catch(Error e){
      System.err.print(e);
      rightCamera = Optional.empty();
    }
  }

  public void updateOdometry(){
    if(rightCamera.isPresent()){
      updateCameraSideOdometry(rightEstimator, rightCamera.get());
    }

    if(leftCamera.isPresent()){
      updateCameraSideOdometry(leftEstimator, leftCamera.get());
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
    if( estimatedPose.isEmpty() ){
      currentStdDevs = singleTagStdDevs;
    }
    else{
      var estimatedStdDevs = singleTagStdDevs;
      int numTags = 0;
      double avgDistance = 0.0;

      for(var tag : targets){
        var tagPose = rightEstimator.getFieldTags().getTagPose( tag.getFiducialId() );
        if( tagPose.isEmpty() ) continue;
        numTags++;
        avgDistance += tagPose
          .get()
          .toPose2d()
          .getTranslation()
          .getDistance( estimatedPose.get().estimatedPose.toPose2d().getTranslation() );
      }

      if (numTags == 0){
        currentStdDevs = singleTagStdDevs;
      }
      else{
        avgDistance /= numTags;

        if( numTags>1 ){ estimatedStdDevs = multiTagStdDevs; }
        
        if( numTags == 1 && avgDistance > 4 ){
          estimatedStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        }
        else{
          estimatedStdDevs = estimatedStdDevs.times( 1+(avgDistance*avgDistance/30) );
        }
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

    SmartDashboard.putBoolean("vision/rightCameraPresent", rightCamera.isPresent());
    SmartDashboard.putBoolean("vision/leftCameraPresent", leftCamera.isPresent());
  }
}
