package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Swerve.Swerve.SwerveInputs;

public class FieldBehaviour{
    public boolean isPassing = false;
    Field2d field = new Field2d();

    public FieldBehaviour(){
        SmartDashboard.putData("FieldBehavior/field",field);
    }


    // Track our field locations, in meters
    public double[] trenchX = new double[]{0,0,0,0};
    public double[] trenchY = new double[]{0,0,0,0};

    public double[] bumpX = new double[]{0,0,0,0};
    public double[] bumpY = new double[]{0,0,0,0};

    public Rotation2d trenchRotation;
    public Rotation2d bumpRotation; 

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
    }
        
    private SwerveInputs avoidWallsY(Pose2d botpose, BoundingBox box){
        var inputs = new SwerveInputs();
        //TODO: Add the appropriate logic
        // if(botpose.getY()>box.upper.getY()/2) inputs.ty = 0.1;
        // if(botpose.getY()>box.lower.getY()/2) inputs.ty = -0.1;
        return inputs;
    }

    private SwerveInputs turnForBump(Pose2d botpose, BoundingBox box){
        var inputs = new SwerveInputs();
        //TODO: Add the appropriate logic
        // if(botpose.getY()>box.upper.getY()/2) inputs.ty = 0.1;
        // if(botpose.getY()>box.lower.getY()/2) inputs.ty = -0.1;
        return inputs;
    }



    public BoundingBox blueLowerTrench = new BoundingBox(trenchX[0],trenchY[0], trenchX[1],trenchY[1]);
    public BoundingBox blueUpperTrench = new BoundingBox(trenchX[0],trenchY[2], trenchX[1],trenchY[3]);
    public BoundingBox redLowerTrench = new BoundingBox(trenchX[2],trenchY[0], trenchX[3],trenchY[1]);
    public BoundingBox redUpperTrench = new BoundingBox(trenchX[2],trenchY[2], trenchX[3],trenchY[3]);
        
    public BoundingBox blueLowerBump = new BoundingBox(trenchX[0],trenchY[0], trenchX[1],trenchY[1]);
    public BoundingBox blueUpperBump = new BoundingBox(trenchX[0],trenchY[2], trenchX[1],trenchY[3]);
    public BoundingBox redLowerBump = new BoundingBox(trenchX[2],trenchY[0], trenchX[3],trenchY[1]);
    public BoundingBox redUpperBump = new BoundingBox(trenchX[2],trenchY[2], trenchX[3],trenchY[3]);        

    private SwerveInputs changeDriveTrainBehaviour(Pose2d robotPosition){
        if(blueUpperTrench.contains(robotPosition)){
           return avoidWallsY(robotPosition, blueUpperTrench);

        } else if(redUpperTrench.contains(robotPosition)){
           return avoidWallsY(robotPosition, redUpperTrench);

        } else if(blueUpperBump.contains(robotPosition)){
           return getSwerveBumpInputs(robotPosition);

        } else if(redUpperBump.contains(robotPosition)){
           return getSwerveBumpInputs(robotPosition);

        } else if(blueLowerTrench.contains(robotPosition)){
           return avoidWallsY(robotPosition, blueLowerTrench);

        } else if(redLowerTrench.contains(robotPosition)){
           return avoidWallsY(robotPosition, redLowerTrench);

        } else if(blueLowerBump.contains(robotPosition)){
           return getSwerveBumpInputs(robotPosition);

        } else if(redLowerBump.contains(robotPosition)){
           return getSwerveBumpInputs(robotPosition);
           
        } else {
            return new SwerveInputs();
        }
    }

    //TODO Get Position and Define Field Pos
    //TODO Make changeDriveTrainBehaviour do stuffs

    public SwerveInputs getSwerveInputs(Pose2d robotPos){
        var response = changeDriveTrainBehaviour(robotPos);
        
                // field.getObject("testpose").setPose(new Pose2d());

        field.setRobotPose(robotPos);
        return response;
    }

    private SwerveInputs getSwerveBumpInputs(Pose2d robotPose){
        var bumpSwerveInputs = new SwerveInputs();
        bumpSwerveInputs.r += 45.0;
        return bumpSwerveInputs;

    }

    private SwerveInputs getTrenchActions(Pose2d robotPose){
        return new SwerveInputs();

    }

    private boolean getRetractHood(){
        return false;
    }

    

    private int boxserialnumber=0;
    public void plotBoundingBox(BoundingBox box){
        field.getObject(String.format("box%n", boxserialnumber)).setPoses(
            new Pose2d(box.lower.getX(),box.lower.getY(),new Rotation2d()),
            //other ones
            new Pose2d(box.lower.getX(),box.lower.getY(),new Rotation2d())
        );
        boxserialnumber++;
    }
}
