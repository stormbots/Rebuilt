// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Spindexer;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public class SpindexerVisual{
    Mechanism2d mech = new Mechanism2d(28, 28);

    MechanismRoot2d root = mech.getRoot("DyeRotor", 14, 14);
    MechanismLigament2d spindexerArm = root.append(new MechanismLigament2d("SpindexerArm", 3, 0));

    MechanismLigament2d upGoerOffset = spindexerArm.append(new MechanismLigament2d("UpGoerOffset", 2, 90));
    MechanismLigament2d upGoerMech = upGoerOffset.append(new MechanismLigament2d("UpGoer", 6, 90));


    //Flair to make it more recognizable
    MechanismLigament2d flair1 = spindexerArm.append(new MechanismLigament2d("spinDexerArm2", 3, 45));
    MechanismLigament2d flair2 = flair1.append(new MechanismLigament2d("spinDexerArm3", 3, 20));

    public SpindexerVisual(){
        var barWeight = 5;

        upGoerMech.setColor(new Color8Bit(Color.kOrange));
        upGoerMech.setLineWeight(barWeight + 5);

        upGoerOffset.setColor(new Color8Bit(Color.kBlack));
        upGoerOffset.setLineWeight(0);

        spindexerArm.setColor(new Color8Bit(Color.kWhite));
        spindexerArm.setLineWeight(barWeight);

        flair1.setColor(new Color8Bit(Color.kWhite));
        flair1.setLineWeight(barWeight);

        flair2.setColor(new Color8Bit(Color.kWhite));
        flair2.setLineWeight(barWeight);

        SmartDashboard.putData("mechanism/Spindexer", mech); 
    }

    public void update(double angle, double upGoerVelocity){
        spindexerArm.setAngle(angle);
        if(upGoerVelocity > 0){
            upGoerMech.setColor(new Color8Bit(Color.kGreen));
        } else if(upGoerVelocity < .1 && upGoerVelocity > -.1){
             upGoerMech.setColor(new Color8Bit(Color.kOrange));
        } else {
            upGoerMech.setColor(new Color8Bit(Color.kRed));
        }
    }
}
