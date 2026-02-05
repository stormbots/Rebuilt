// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class Constants {
    public static class Field{
        public static Translation2d blueHub=new Translation2d(4.6,4);
        public static Translation2d redHub=new Translation2d(11.97,4);
    }

    public static class Shooter{
        public static Translation3d botToTurretOffset = new Translation3d(0, 0, Inches.of(20).in(Meter));
    }

    public static class Bumpers{
        public static Distance width = Inches.of(22);
        public static Distance length = Inches.of(22);
        public static Distance height = Inches.of(6);
    }

    public static class Intake{
        public static Distance reach = Inches.of(6);
        public static Distance width = Inches.of(22);
    }

}
