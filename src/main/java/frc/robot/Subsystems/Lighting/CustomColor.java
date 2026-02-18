// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Lighting;

/** You can pry this from my cold dead hands  */
public class CustomColor {
    int red;
    int green;
    int blue;


    /** Constructs a new CustomColor
     * @param red Red value (0-255)
     * @param green Green value (0-255)
     * @param blue Blue value (0-255) */
    public CustomColor(int red, int green, int blue){
      this.red =red;
      this.green = green;
      this.blue = blue;
    }

    public CustomColor(String hex){
      this.red = toRed(hex);
      this.green = toGreen(hex);
      this.blue = toBlue(hex);
    }

    /** Constructs a default CustomColor (black)  */
    public CustomColor(){
      this.red =0;
      this.green = 0;
      this.blue = 0;
    }

    private int toInteger(){
      return (255 << 24) | (red << 16) | (green << 8) | blue;
    }

    private int toRed(String hex){
      return Integer.valueOf(hex.substring(0,2),16);
    }

    private int toGreen(String hex){
      return Integer.valueOf(hex.substring(2,4),16);
    }

    private int toBlue(String hex){
      return Integer.valueOf(hex.substring(4,6),16);
    }

    public String getHex(){
      String hex = Integer.toString(toInteger() & 0x00ffffff, 16);
      if (red == 0){
        hex = "00"+hex;
      }
      if (green == 0 && red == 0){
        hex = "00"+hex;
      }
      return hex;
    }

    public static final CustomColor kPink = new CustomColor(255, 28, 206);
    public static final CustomColor kWhite = new CustomColor(255, 255, 255);
    public static final CustomColor kBabyBlue = new CustomColor(38, 14, 255);
    public static final CustomColor kRed = new CustomColor(255, 0, 0);
    public static final CustomColor kOrange = new CustomColor(255, 140, 0);
    public static final CustomColor kYellow = new CustomColor(255, 215, 0);
    public static final CustomColor kGreen = new CustomColor(0, 255, 0);
    public static final CustomColor kBlue = new CustomColor(0, 0, 255);
    public static final CustomColor kPurple = new CustomColor(255, 0, 255);
    public static final CustomColor kBlack = new CustomColor(0, 0, 0);
    public static final CustomColor kGrey = new CustomColor(119, 119, 119);
    public static final CustomColor kMagenta = new CustomColor(255, 0, 111);
    public static final CustomColor kLightGreen = new CustomColor(159, 255, 133); 
}

