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
  private boolean backLeftHasTarget = false;
  private boolean backRightHasTarget = false;
  private boolean frontLeftHasTarget = false;
  private boolean frontRightHasTarget = false;

  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  private Optional<PhotonCamera> rightCamera = Optional.empty();
  private Optional<PhotonCamera> frontLeftCamera = Optional.empty();
  private Optional<PhotonCamera> backLeftCamera = Optional.empty();
  private Optional<PhotonCamera> backRightCamera = Optional.empty();

  private Matrix<N3, N1> currentStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
  private Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(.75, .75, 4);
  private Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  private Transform3d rightCameraToCenter = new Transform3d(
    new Translation3d(
    //Translation measured on bot
      Inch.of(9.458).in(Meters), 
      Inch.of(-9.458).in(Meters), 
      Inch.of(21.122).in(Meters)), 
    new Rotation3d(Math.toRadians(0.0), Math.toRadians(-17.5), Math.toRadians(-45.0))
  );
  private Transform3d frontLeftCameraToCenter = new Transform3d(
    new Translation3d(
      Inch.of(9.458).in(Meters), 
      Inch.of(9.458).in(Meters), 
      Inch.of(21.122).in(Meters)),
    new Rotation3d(0.0, Math.toRadians(-17.5), Math.toRadians(45.0))
  );
  private Transform3d backLeftCameraToCenter = new Transform3d(
    new Translation3d(
      Inch.of(.359).in(Meters),
      Inch.of(12.717).in(Meters),
      Inch.of(21.330).in(Meters)
    ),
    new Rotation3d(0.0, Math.toRadians(-17.5), Math.toRadians(137.5))
  );
  private Transform3d backRightCameraToCenter = new Transform3d(
    new Translation3d(
      Inch.of(.359).in(Meters),
      Inch.of(-12.717).in(Meters),
      Inch.of(21.330).in(Meters)
    ),
    new Rotation3d(0.0, Math.toRadians(-17.5), Math.toRadians(-137.5))
  );

  private Field2d visionField2d = new Field2d();
  private PhotonPoseEstimator frontRightEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, rightCameraToCenter);
  private PhotonPoseEstimator frontLeftEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, frontLeftCameraToCenter);
  private PhotonPoseEstimator backLeftEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, backLeftCameraToCenter);
  private PhotonPoseEstimator backRightEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, backRightCameraToCenter);

  /** Creates a new Photonvision.
   *  @param swerve 
   */
  public Photonvision(Swerve swerve) {
    this.swerve = swerve;
    SmartDashboard.putData("visionfield", visionField2d);

    // TODO: Fix this! If one camera throws an error, we have no cameras
    try{
      rightCamera = Optional.of(new PhotonCamera("FrontRight"));
    }
    catch(Error e){
      System.err.print(e);
      rightCamera = Optional.empty();
    }

    try{
      frontLeftCamera = Optional.of(new PhotonCamera("FrontLeft"));
    }
    catch(Error e){
      System.err.print(e);
      frontLeftCamera = Optional.empty();
    }

    try{
      backLeftCamera = Optional.of(new PhotonCamera("BackLeft"));
    }
    catch(Error e){
      System.err.print(e);
      backLeftCamera = Optional.empty();
    }
    try{
      backRightCamera = Optional.of(new PhotonCamera("BackRight"));
    }
    catch(Error e){
      System.err.print(e);
      backRightCamera = Optional.empty();
    }

  }

  public void updateOdometry(){
    if(rightCamera.isPresent()){
      updateCameraSideOdometry(frontRightEstimator, rightCamera.get());
    }

    if(frontLeftCamera.isPresent()){
      updateCameraSideOdometry(frontLeftEstimator, frontLeftCamera.get());
    }

    if(backLeftCamera.isPresent()){
      updateCameraSideOdometry(backLeftEstimator, backLeftCamera.get());
    }
    
    if(backRightCamera.isPresent()){
      updateCameraSideOdometry(backRightEstimator, backRightCamera.get());
    }
  }

  private void updateCameraSideOdometry(PhotonPoseEstimator poseEstimator, PhotonCamera camera){
    Optional<EstimatedRobotPose> visionEstimate = Optional.empty();
    for(var result : camera.getAllUnreadResults()){
      visionEstimate = poseEstimator.estimateCoprocMultiTagPose(result);
      if (visionEstimate.isEmpty()){
        visionEstimate = poseEstimator.estimateLowestAmbiguityPose(result);
        if(poseEstimator.equals(frontRightEstimator)){
          frontRightHasTarget = false;
        }
        else if(poseEstimator.equals(frontLeftEstimator)){
          frontLeftHasTarget = false;
        }
        else if(poseEstimator.equals(backRightEstimator)){
          backRightHasTarget = false;
        }
        else if(poseEstimator.equals(backLeftEstimator)){
          backLeftHasTarget = false;
        };
      }
      updateEstimationStdDevs(visionEstimate, result.getTargets());
    }

    visionEstimate.ifPresent(
      est ->{
        var estimatedStdDevs = getEstimationStdDevs();

        swerve.swerveDrive.addVisionMeasurement(est.estimatedPose.toPose2d(),est.timestampSeconds, estimatedStdDevs);
        visionField2d.getObject(camera.getName()).setPose(est.estimatedPose.toPose2d());
        if(poseEstimator.equals(frontRightEstimator)){
          frontRightHasTarget = true;
        }
        else if(poseEstimator.equals(frontLeftEstimator)){
          frontLeftHasTarget = true;
        }
        else if(poseEstimator.equals(backRightEstimator)){
          backRightHasTarget = true;
        }
        else if(poseEstimator.equals(backLeftEstimator)){
          backLeftHasTarget = true;
        };
      }
    );
  }

  public boolean hasTarget(){
    return frontLeftHasTarget || frontRightHasTarget || backLeftHasTarget || backRightHasTarget;
  }

  public boolean doesNotHaveTarget(){
    return !hasTarget();
  }

  public void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets){
    if( estimatedPose.isEmpty() ){
      currentStdDevs = singleTagStdDevs;
    } else{
      var estimatedStdDevs = singleTagStdDevs;
      int numTags = 0;
      double avgDistance = 0.0;

      for(var tag : targets){
        var tagPose = frontRightEstimator.getFieldTags().getTagPose(tag.getFiducialId());
        if( tagPose.isEmpty() ) continue;
        numTags++;
        avgDistance += tagPose
          .get()
          .toPose2d()
          .getTranslation()
          .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation() );
      }

      if (numTags == 0){
        currentStdDevs = singleTagStdDevs;
      } else{
        avgDistance /= numTags;

        if(numTags > 1){
          estimatedStdDevs = multiTagStdDevs;
        } else if(numTags == 1 && avgDistance > 4){
          estimatedStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else{
          estimatedStdDevs = estimatedStdDevs.times(1 + (avgDistance*avgDistance / 30));
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
    SmartDashboard.putBoolean("vision/leftCameraPresent", frontLeftCamera.isPresent());
    SmartDashboard.putBoolean("vision/backLeftCameraPresent", backLeftCamera.isPresent());
    SmartDashboard.putBoolean("vision/backRightCameraPresent", backRightCamera.isPresent());

    SmartDashboard.putBoolean("vision/hasTarget", hasTarget());
    SmartDashboard.putBoolean("vision/doesNotHaveTarget", doesNotHaveTarget());
  }
}
