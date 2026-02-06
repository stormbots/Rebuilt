// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public class IntakeVisualizer {
    double barlength = 24;
    double intakeArmLength = 7; //inches


    public Mechanism2d mech = new Mechanism2d(36, 72);
    MechanismRoot2d root = mech.getRoot("IntakeRoot", 18, 6);

    MechanismLigament2d intakeArm = root.append(new MechanismLigament2d("IntakeArm", intakeArmLength, 0));
    MechanismLigament2d roller = intakeArm.append(new MechanismLigament2d("RotatorRelative", 13, 0));

    public IntakeVisualizer() {
      var barweight = 10;

      intakeArm.setColor(new Color8Bit(Color.kGray));
      intakeArm.setLineWeight(barweight);

      roller.setLength(barweight/2);
      roller.setColor(new Color8Bit(Color.kOrange));

      SmartDashboard.putData("mechanism/intake", mech);
    }

    public void update(
        double angle,
        double rollerPosition,
        double rollerVelocity
        ) {
      intakeArm.setAngle(angle);
      roller.setAngle(rollerPosition/100);

      if(rollerVelocity==0)roller.setColor(new Color8Bit(Color.kOrange));
      if(rollerVelocity>0)roller.setColor(new Color8Bit(Color.kGreen));
      if(rollerVelocity<0)roller.setColor(new Color8Bit(Color.kRed));
    }
  }