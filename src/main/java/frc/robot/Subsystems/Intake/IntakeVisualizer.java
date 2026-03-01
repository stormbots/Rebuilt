// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Intake;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public class IntakeVisualizer {
  double intakeArmLength = 7; //inches
  double rollerDiameter=1; //inches

  public Mechanism2d mech = new Mechanism2d(15+2+7+5, 15);
  MechanismRoot2d root = mech.getRoot("IntakeRoot", 18, 6);

  MechanismLigament2d intakeArm = root.append(new MechanismLigament2d("IntakeArm", intakeArmLength, 0));
  MechanismLigament2d roller = intakeArm.append(new MechanismLigament2d("Roller", rollerDiameter/2, 0));
  
  //This is just to help provide a square, nice rotation
  MechanismLigament2d rollerSecondary = intakeArm.append(new MechanismLigament2d("RollerSecondary", -rollerDiameter/2, 0));

  public IntakeVisualizer() {
    var barweight = 10;

    intakeArm.setColor(new Color8Bit(Color.kGray));
    intakeArm.setLineWeight(barweight);

    roller.setLineWeight(barweight*2);
    rollerSecondary.setLineWeight(barweight*2);
    
    SmartDashboard.putData("mechanism/intake", mech);
  }

  public void update(
      Angle angle,
      Angle rollerPosition,
      AngularVelocity rollerVelocity
    ) {
    intakeArm.setAngle(angle.in(Degrees));

    roller.setAngle(rollerPosition.in(Degrees));
    rollerSecondary.setAngle(rollerPosition.in(Degrees));

    var rollercolor = Color.kOrange;
    if(rollerVelocity.in(Units.DegreesPerSecond) > 0.05){
      rollercolor = Color.kGreen;
    }
    else if(rollerVelocity.in(Units.DegreesPerSecond) < 0.05){
      rollercolor = Color.kRed;
    }
    roller.setColor(new Color8Bit(rollercolor));
    rollerSecondary.setColor(new Color8Bit(rollercolor));
  }
}