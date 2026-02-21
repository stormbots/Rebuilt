// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter.Turret;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public class TurretVisual {

    private Turret turret;
    

    // Create the basic turret facing
    public double width=6;
    public double length=7;

    Mechanism2d mechanism = new Mechanism2d(24, 24);
    MechanismRoot2d root = mechanism.getRoot("TurretRoot", 12, 12);
    MechanismLigament2d pivot = root.append(new MechanismLigament2d("Turret", length/2.0, 0));

    // Add graphics on it
    MechanismLigament2d frontBarRight = pivot.append(new MechanismLigament2d("FrontBarRight", width/2.0, 90));
    MechanismLigament2d rightBar = frontBarRight.append(new MechanismLigament2d("RightBar", length, 90));
    MechanismLigament2d backBar = rightBar.append(new MechanismLigament2d("BackBar", width, 90));
    MechanismLigament2d leftBar = backBar.append(new MechanismLigament2d("BackBar", length, 90));
    MechanismLigament2d frontBarLeft = leftBar.append(new MechanismLigament2d("FrontBarLeft", width/2.0, 90));

    //Set the absolute limits of the system
    MechanismRoot2d limits = mechanism.getRoot("Limits", 12, 12);
    // Angle minAngle = Turret.kMaximumRotationRange.minus(Turret.kRotationToRobotForward);
    Angle minAngle = Degrees.of(-Turret.kMaxRotation);
    Angle maxAngle = Degrees.of(Turret.kMaxRotation);
    Angle turretRange = maxAngle.minus(minAngle);
    // MechanismLigament2d minBar = root.append(new MechanismLigament2d(
    //     "LimitCCW",7,  minAngle.in(Degrees),1,new Color8Bit(Color.kRed)
    // ));
    // MechanismLigament2d maxBar = root.append(new MechanismLigament2d(
    //     "LimitCW",7, maxAngle.in(Degrees),1,new Color8Bit(Color.kGreen)
    // ));

    double encScale=1/36.0;
    MechanismRoot2d realEncoder = mechanism.getRoot("Encoder", 12, 20);
    //Vertical lines because reasons
    MechanismLigament2d verticalPlaceOffset1 = root.append(new MechanismLigament2d(
        "VOffset1",180*encScale,  90,2,new Color8Bit(Color.kAzure)
    ));
    //Draw the relative encoder across the whole sliding scale of possibility
    MechanismLigament2d encMin = verticalPlaceOffset1.append(new MechanismLigament2d(
        "EncMin",minAngle.in(Degrees)*encScale,  90,1,new Color8Bit(Color.kWhite)
    ));
    MechanismLigament2d encMax = encMin.append(new MechanismLigament2d(
        "EncMax",turretRange.in(Degrees)*encScale,  180,2,new Color8Bit(Color.kWhite)
    ));
    MechanismLigament2d encReadingOffset = verticalPlaceOffset1.append(new MechanismLigament2d(
        "EncReadingOffset",0,  -90,4,new Color8Bit(Color.kOrange)
    ));
    MechanismLigament2d encReading = encReadingOffset.append(new MechanismLigament2d(
        "EncoderReading",20*encScale,  90,6,new Color8Bit(Color.kRed)
    ));

    //Visualize a 0..360 range in the current state
    MechanismLigament2d verticalPlaceOffset2 = verticalPlaceOffset1.append(new MechanismLigament2d(
        "VOffset2",90*encScale,  0,2,new Color8Bit(Color.kBlue)
    ));
    MechanismLigament2d rangeMin = verticalPlaceOffset2.append(new MechanismLigament2d(
        "RangeMin",minAngle.in(Degrees)*encScale,  90,1,new Color8Bit(Color.kWhite)
    ));
    MechanismLigament2d rangeOffset = rangeMin.append(new MechanismLigament2d(
        "RangeOffset",0,  180,2,new Color8Bit(Color.kWhite)
    ));
    MechanismLigament2d range = rangeOffset.append(new MechanismLigament2d(
        "Range",360*encScale,  0,4,new Color8Bit(Color.kGreen)
    ));


    public void makeBoring(MechanismLigament2d segment){
        segment.setColor(new Color8Bit(Color.kGray));
        segment.setLineWeight(2);
    }

    public TurretVisual(Turret turret){
        //TODO: Build the Mechanism2D reference package
        SmartDashboard.putData("mechanism/turret",mechanism);
        this.turret = turret;

        makeBoring(frontBarRight);
        makeBoring(rightBar);
        makeBoring(backBar);
        makeBoring(leftBar);
        makeBoring(frontBarLeft);
    }



    public void update(Angle angle) {
        // double degrees = turret.getAngle().in(Degrees);
        pivot.setAngle(angle.in(Degrees)%360);
        encReadingOffset.setLength(angle.in(Degrees)*encScale);
    }

}
