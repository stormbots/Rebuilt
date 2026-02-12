// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Climber;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public class ClimberVisual {

    public final static double kWidth = 22;
    public final static double kBotHeight = 24;
    public final static double kSpacing = 4;

    public final static double kLinewidth=4;
    public final static Color8Bit gray=new Color8Bit(Color.kDarkGray);
    public final static Color8Bit pink=new Color8Bit(Color.kPink);

    public Mechanism2d mech = new Mechanism2d(22, 40);
    public MechanismRoot2d root = mech.getRoot("root", kWidth/2, 0);

    public MechanismLigament2d stage1Offset = root.append(new MechanismLigament2d("s1offset", kSpacing/2, 180,kLinewidth,gray));
    public MechanismLigament2d stage1base = stage1Offset.append(new MechanismLigament2d("s1base", kBotHeight, -90,kLinewidth,gray));
    public MechanismLigament2d stage1Actuator = stage1base.append(new MechanismLigament2d("s1base", 0, 0,kLinewidth,gray));
    public MechanismLigament2d stage1Hook = stage1Actuator.append(new MechanismLigament2d("s1Actuator", kSpacing/2, -90,kLinewidth,pink));
    public MechanismLigament2d stage1Flange = stage1base.append(new MechanismLigament2d("s1flange", -0.5, 0,kLinewidth*3,gray));


    public MechanismLigament2d stage2Offset = root.append(new MechanismLigament2d("s2offset", kSpacing/2, 0,kLinewidth,gray));
    public MechanismLigament2d stage2base = stage2Offset.append(new MechanismLigament2d("s2base", kBotHeight-Climber.kStage2Range.in(Inch), 90,kLinewidth,gray));
    public MechanismLigament2d stage2Actuator = stage2base.append(new MechanismLigament2d("s2base", 0, 0,kLinewidth,gray));
    public MechanismLigament2d stage2Hook = stage2Actuator.append(new MechanismLigament2d("s2Actuator", kSpacing/2, 90,kLinewidth,pink));
    public MechanismLigament2d stage2Flange = stage2base.append(new MechanismLigament2d("s2flange", -0.5, 0,kLinewidth*3,gray));




    public ClimberVisual(){
        SmartDashboard.putData("mechanism/Climber",mech);
        stage1Actuator.setColor(new Color8Bit(Color.kGreen));
        stage2Actuator.setColor(new Color8Bit(Color.kRed));
    }



    // public MechanismLigament2d stage2Offset = root.append(new MechanismLigament2d("s2offset", kSpacing/2, 180));


    public void update(Distance stage1Position, Distance stage2Position){
        stage1Actuator.setLength(stage1Position.in(Inches));
        stage2Actuator.setLength(stage2Position.in(Inches));
    };



}
