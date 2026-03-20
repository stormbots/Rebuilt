// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Climber;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Climber.ClimberExtension.ClimberExtension;
import frc.robot.Subsystems.Climber.Grabber.Grabber;
import frc.robot.Subsystems.Swerve.Swerve.SwerveInputs;

/** Add your docs here. */
public class Climber extends SubsystemBase {
    public static Distance kStage1Range = Inches.of(8.25);
    public static Distance kStage2Range = Inches.of(20);

    private Rangefinders rangefinders = new Rangefinders();  
      
    ClimberExtension stage1 = new ClimberExtension(
        "Stage1", 19, true, kStage1Range
    );
    ClimberExtension stage2 = new ClimberExtension(
        "Stage2", 20, true, kStage2Range
    );

    Grabber grabber = new Grabber();

    public ClimberVisual visual = new ClimberVisual();

    public Climber(){

        // new Trigger(()->stage1.isHomed() && stage2.isHomed())
        // .whileTrue(
        //     stage2.setHeight(kStage2Range.minus(Inches.of(3)))
        //     .withName("PresetStage2")
        // );
    }

    @Override
    public void periodic(){
        visual.update(stage1.getHeight(), stage2.getHeight());
        SmartDashboard.putNumber("climber/position", (stage1.getHeight().in(Inches)));
    }


    public Command goHome(){
        return Commands.parallel(
            stage1.goHome(),
            // stage2.goHomes(),
            grabber.retract(),
            Commands.none()
        );
    }

    public Command setStage1Voltage(double voltage){
        return stage1.setVoltage(voltage);
    }

    public Command setStage2Voltage(double voltage){
        return stage2.setVoltage(voltage);
    }

    public Command prepareForClimbL1(){
        return Commands.parallel(
            stage1.setHeight(Inches.of(8.25)),
            grabber.retractPartial()
        )
        // .until(grabber.isRetracted.and(stage1.isAtTargetPosition))
        .finallyDo(stage1::stopMotor)
        .withName("PrepareToClimb");
    }

    public Command climbL1(){
        
        return Commands.sequence(
            grabber.grab().until(grabber.isPossiblyConnected).withTimeout(0.5),
            new WaitCommand(0.25),
            stage1.setHeight(Inches.of(2.5))
        )
        .finallyDo(stage1::stopMotor)
        .withName("Climb");
    }

    public Command declimbL1() {
        return Commands.sequence(
            stage1.setHeight(Inches.of(6))
            .until(stage1.isAtTargetPosition),
            grabber.retract()
        )
        .finallyDo(stage1::stopMotor)
        .withName("declimb")
        ;
        //at end, not sure if actually away from post
    }

    public Command stow(){
        return Commands.parallel(
            stage1.setHeight(Inches.of(0)),
            // stage2.setHeight(Inches.of(10)),
            grabber.retract()
        )
        // .until(grabber.isRetracted.and(stage1.isAtTargetPosition))
        .andThen(Commands.idle().withTimeout(1))
        .finallyDo(stage1::stopMotor)
        .withName("stow");
    }

    public SwerveInputs generateSwerveInputs(Pose2d botpose){
        double facingAngleDegrees = botpose.getRotation().getDegrees();
        if(facingAngleDegrees < 180  &&  facingAngleDegrees > 0){ facingAngleDegrees = 90; }
        else{ facingAngleDegrees = -90;}

        return rangefinders.generateInputs(facingAngleDegrees);
    }

    public boolean isLinedUpWithL1(){
        return rangefinders.isLinedUpL1();
    }
}
