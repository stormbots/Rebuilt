package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.Swerve.Swerve.SwerveInputs;

public class FieldBehaviour{

    /** Supplemental class to wrap poses up nicely */
    public static class BoundingBox{
        public Translation2d lower;
        public Translation2d upper ;

        public BoundingBox(double lowerX, double lowerY, double upperX, double upperY){
            this.lower = new Translation2d(lowerX,lowerY);
            this.upper = new Translation2d(upperX,upperY);
        }
        
        public boolean contains(Translation2d pose){
            if(pose.getX() < lower.getX()) return false;
            if(pose.getX() > upper.getX()) return false;
            if(pose.getY() < lower.getY()) return false;
            if(pose.getY() > upper.getY()) return false;
            return true;
        }

        public boolean contains(Pose2d pose){
            return contains(pose.getTranslation());
        }

        public List<Pose2d> toPoses(){
            return List.of(
                new Pose2d(lower.getX(),lower.getY(),new Rotation2d()),
                new Pose2d(upper.getX(),lower.getY(),new Rotation2d()),
                new Pose2d(upper.getX(),upper.getY(),new Rotation2d()),
                new Pose2d(lower.getX(),upper.getY(),new Rotation2d()),
                new Pose2d(lower.getX(),lower.getY(),new Rotation2d())
            );
        }
    }


    public boolean isPassing = false;
    Field2d field = new Field2d();

    public FieldBehaviour(){
        SmartDashboard.putData("FieldBehaviour/field",field);
        field.getObject("TestPose").setPose(new Pose2d(7, 5, new Rotation2d(90)));

        if(DriverStation.isFMSAttached()) return; //Don't do debug things
        new Trigger(DriverStation::isEnabled)
        .whileTrue(Commands.run(()->{
            SmartDashboard.putBoolean("FieldBehaviour/retractHood", getRetractHood(field.getObject("TestPose").getPose()));
            SmartDashboard.putBoolean("FieldBehaviour/climber", getClimber(field.getObject("TestPose").getPose()));
        }));

        field.getObject("blthb").setPoses(blueLowerTrenchHoodBox.toPoses());
        field.getObject("buthb").setPoses(blueUpperTrenchHoodBox.toPoses());
        field.getObject("rlthb").setPoses(redLowerTrenchHoodBox.toPoses());
        field.getObject("ruthb").setPoses(redUpperTrenchHoodBox.toPoses());

        field.getObject("cb").setPoses(climbingBlue.toPoses());
        field.getObject("cr").setPoses(climbingRed.toPoses());
    }


    // Track our field locations, in meters
    private double blueCenter = 4.65;
    private double redCenter = 11.887;

    //generate X coordinate offsets from centerline of obstacles
    private double trenchHoodOffset = .65;

    public double[] centerX = new double[]{blueCenter, redCenter};
    public double[] trenchY = new double[]{0, 1.2, 6.7, 8};

    public double[] bumpY = new double[]{1.7, 3.3, 4.6, 6.3};

    //TODO: Change bump and trench boxes to be important for stuff
    public BoundingBox blueLowerTrench = new BoundingBox(centerX[0], trenchY[0], centerX[0], trenchY[1]);
    public BoundingBox blueUpperTrench = new BoundingBox(centerX[0], trenchY[2], centerX[0], trenchY[3]);
    public BoundingBox redLowerTrench = new BoundingBox(centerX[1], trenchY[0], centerX[1], trenchY[1]);
    public BoundingBox redUpperTrench = new BoundingBox(centerX[1], trenchY[2], centerX[1], trenchY[3]);
        
    public BoundingBox blueLowerBump = new BoundingBox(centerX[0],trenchY[0], centerX[0],trenchY[1]);
    public BoundingBox blueUpperBump = new BoundingBox(centerX[0],trenchY[2], centerX[0],trenchY[3]);
    public BoundingBox redLowerBump = new BoundingBox(centerX[1],trenchY[0], centerX[1],trenchY[1]);
    public BoundingBox redUpperBump = new BoundingBox(centerX[1],trenchY[2], centerX[1],trenchY[3]);   
    
    public BoundingBox blueLowerTrenchHoodBox = new BoundingBox(centerX[0] - trenchHoodOffset, trenchY[0], centerX[0] + trenchHoodOffset, trenchY[1]);
    public BoundingBox blueUpperTrenchHoodBox = new BoundingBox(centerX[0] - trenchHoodOffset, trenchY[2], centerX[0] + trenchHoodOffset, trenchY[3]);
    public BoundingBox redUpperTrenchHoodBox = new BoundingBox(centerX[1] - trenchHoodOffset, trenchY[0], centerX[1] + trenchHoodOffset, trenchY[1]);
    public BoundingBox redLowerTrenchHoodBox = new BoundingBox(centerX[1] - trenchHoodOffset, trenchY[2], centerX[1] + trenchHoodOffset, trenchY[3]);

    public BoundingBox climbingBlue = new BoundingBox(.912, 3.054, 1.715, 4.375);
    public BoundingBox climbingRed = new BoundingBox(14.82, 3.545, 15.615, 5.036);

    public ArrayList<BoundingBox> swerveTrenches = new ArrayList<>(){{
        add(blueLowerTrench);
        add(blueUpperTrench);
        add(redLowerTrench);
        add(redUpperTrench);
    }};

   public ArrayList<BoundingBox> bumpList = new ArrayList<>(){{
        add(blueLowerBump);
        add(blueUpperBump);
        add(redLowerBump);
        add(redUpperBump);
    }};
    
    public ArrayList<BoundingBox> hoodTrenches = new ArrayList<>(){{
        add(blueLowerTrenchHoodBox);
        add(blueUpperTrenchHoodBox);
        add(redUpperTrenchHoodBox);
        add(redLowerTrenchHoodBox);
    }};

    public ArrayList<BoundingBox> climbingList = new ArrayList<>(){{
        add(climbingBlue);
        add(climbingRed);
    }};

    private SwerveInputs avoidWallsY(Pose2d botpose, BoundingBox box){
        var inputs = new SwerveInputs();
        //TODO: Add the appropriate logic
        // if(botpose.getY()>box.upper.getY()/2) inputs.ty = 0.1;
        // if(botpose.getY()>box.lower.getY()/2) inputs.ty = -0.1;
        return inputs;
    }

    private SwerveInputs computeForBump(Pose2d botpose, BoundingBox box){
        var inputs = new SwerveInputs();
        //TODO: Add the appropriate logic
        // if(botpose.getY()>box.upper.getY()/2) inputs.ty = 0.1;
        // if(botpose.getY()>box.lower.getY()/2) inputs.ty = -0.1;
        return inputs;
    }



    private SwerveInputs changeDriveTrainBehaviour(Pose2d robotPosition){
        var swerveInputs = new SwerveInputs();
        for(var trench : swerveTrenches){
            if(trench.contains(robotPosition)){
                swerveInputs.add(avoidWallsY(robotPosition, trench));
            }
        }

        for(var bump : bumpList){
            if(bump.contains(robotPosition)){
                swerveInputs.add(computeForBump(robotPosition, bump));
            }
        }

        return swerveInputs;
    }

    public boolean getRetractHood(Pose2d robotPosition){
        for(var trench : hoodTrenches){
            if(trench.contains(robotPosition)){
                return true;
            }
        }
        return false;
    }

    public boolean getClimber(Pose2d robotPosition){
        for(var climber : climbingList){
            if(climber.contains(robotPosition)){
                return true;
            }
        }
        return false;
    }

    public SwerveInputs getSwerveInputs(Pose2d robotPos){
        var response = changeDriveTrainBehaviour(robotPos);
        field.setRobotPose(robotPos);
        return response;
    }
}
