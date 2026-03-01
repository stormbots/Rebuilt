// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Lighting;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Lighting.LedSegment.LedMultiRange;

/** Add your docs here. */
public class LedBase extends SubsystemBase{
    protected int id; //set once
    protected Optional<Integer> start;// set once
    protected Optional<Integer> stop; //set once
    protected CustomColor[] col;
    protected Optional<Integer> fx = Optional.empty();
    protected Optional<Integer> sx = Optional.empty();
    protected Optional<Integer> ix = Optional.empty();
    protected Optional<Integer> c1 = Optional.empty();
    protected Optional<Integer> c2 = Optional.empty();
    protected Optional<Integer> c3 = Optional.empty();
    protected Optional<Boolean> rev = Optional.empty();
    protected Optional<Boolean> on = Optional.empty();
    protected Optional<Integer> bri = Optional.empty();
    protected Optional<Integer> pal = Optional.empty();
    protected LedMultiRange i;
    protected Optional<Boolean> frz = Optional.empty();

    public LedBase(int id, int start, int stop, boolean rev){
        this.id = id;
        this.start = Optional.of(start);
        this.stop = Optional.of(stop);
        this.rev = Optional.of(rev);
    }
}


