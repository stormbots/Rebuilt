// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Lighting;

import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class LedSegmentData extends LedBase{
    
private transient boolean updated = true;
private transient int index;

    public LedSegmentData(int id, int start, int stop, boolean rev){
        super(id, start, stop, rev);
        index = WLED.registerState(updated);
    }

    public void setColor(CustomColor... color){
        this.col = color;
        updated  = true;
    }

    public void setEffect(int fx){
        this.fx = Optional.of(fx);
        updated  = true;
    }

    public void setSpeed(int sx){
        this.sx = Optional.of(sx);
        updated  = true;
    }

    public void setIntensity(int ix){
        this.ix = Optional.of(ix);
        updated  = true;
    }

    public void setCustomSlider1(int c1){
        this.c1 = Optional.of(c1);
        updated  = true;
    }

    public void setCustomSlider2(int c2){
        this.c2 = Optional.of(c2);
        updated  = true;
    }

    public void setCustomSlider3(int c3){
        this.c3 = Optional.of(c3);
        updated  = true;
    }

    public void setReverse(boolean rev){
        this.rev = Optional.of(rev);
        updated  = true;
    }

    public void setOn(boolean on){
        this.on = Optional.of(on);
        updated  = true;
    }

    public void setBrightness(int bri){
        this.bri = Optional.of(bri);
        updated  = true;
    }

    public void setIndividualControl(LedSegment.LedMultiRange i){
        this.i = i;
        updated  = true;
    }

    public void setFreeze(boolean frz){
        this.frz = Optional.of(frz);
        updated  = true;
    }

    public void reset(){
        start =Optional.empty();// set once
        stop = Optional.empty(); //set once
        col = null;
        fx = Optional.empty();
        sx = Optional.empty();
        ix = Optional.empty();
        c1 = Optional.empty();
        c2 = Optional.empty();
        c3 = Optional.empty();
        rev = Optional.empty();
        on = Optional.empty();
        bri = Optional.empty();
        i = null;
        frz = Optional.empty();
        updated = false;
        WLED.updateState(updated, index);
    }

    public boolean isUpdated(){
        return updated;
    }

    @Override
    public void periodic() {
        WLED.updateState(updated, index);
        SmartDashboard.putNumber("index", index);
        SmartDashboard.putBoolean("isUpdated", updated);
    }


}
