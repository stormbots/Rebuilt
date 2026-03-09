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

  /** Constructs a new CustomColor 
   * <p>
   * (untested)
   * @param hex 6-digit hex code*/
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
    // return (255 << 24) | (red << 16) | (green << 8) | blue;
    return (red << 16) | (green << 8) | blue;

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
    // String hex = Integer.toString(toInteger() & 0x00ffffff, 16);
    String hex = Integer.toString(toInteger() & 0xffffff, 16);
    if (red <= 16){
      hex = "0"+hex;
    }
    if (red == 0){
      hex = "0"+hex;
    }
    if (green == 0 && red == 0){
      hex = "00"+hex;
    }
    if (hex.length()<6){
      hex= hex+"0";
    }
    if (hex.length()<6){
      hex= hex+"0";
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
  public static final CustomColor kLightPurple = new CustomColor(255, 233, 255);
  public static final CustomColor kPurple = new CustomColor(255, 0, 255);
  public static final CustomColor kBlack = new CustomColor(0, 0, 0);
  public static final CustomColor kGrey = new CustomColor(119, 119, 119);
  public static final CustomColor kMagenta = new CustomColor(255, 0, 111);
  public static final CustomColor kLightGreen = new CustomColor(159, 255, 133);

  public static final CustomColor kChileBlue = new CustomColor(0, 50, 160);
  public static final CustomColor kArgentinaBlue = new CustomColor(108, 172, 228);
  public static final CustomColor kArgentinaYellow = new CustomColor(255, 184, 28);
  public static final CustomColor kUruguayYellow = new CustomColor(255,205,0);
  public static final CustomColor kParaguayGreen = new CustomColor(28, 204, 0);
  public static final CustomColor kBoliviaRed = new CustomColor(218, 41, 28);
  public static final CustomColor kBoliviaYellow = new CustomColor(248, 230, 0);
  public static final CustomColor kBoliviaGreen = new CustomColor(0, 122, 51);
  public static final CustomColor kEcuadorYellow = new CustomColor(255, 209, 0);
  public static final CustomColor kEcuadorGreen = new CustomColor(14, 156, 31);
  public static final CustomColor kEcuadorBlue = new CustomColor(0, 114, 206);
  public static final CustomColor kBrazilGreen = new CustomColor(0, 151, 57);
  public static final CustomColor kBrazilYellow = new CustomColor(254, 221, 0);
  public static final CustomColor kBrazilBlue = new CustomColor(1, 33, 105);
  public static final CustomColor kVenezuelaYellow = new CustomColor(252, 227, 0);
  public static final CustomColor kVenezuelaBlue = new CustomColor(0, 61, 165);
  public static final CustomColor kGuyanaRed = new CustomColor(239, 51, 64);
  public static final CustomColor kGuyanaGreen = new CustomColor(0, 151, 57);
  public static final CustomColor kPanamaBlue = new CustomColor(13, 68, 168);
  public static final CustomColor kPanamaRed = new CustomColor(218, 18, 26);
  public static final CustomColor kCostaRicaBlue = new CustomColor(0, 32, 91);
  public static final CustomColor kNicaraguaBlue = new CustomColor(0, 103, 198);
  public static final CustomColor kNicaraguaGreen = new CustomColor(2, 222, 149);
  public static final CustomColor kHondurasBlue = new CustomColor(0, 145, 212);
  public static final CustomColor kElSalvadorBlue = new CustomColor(0, 71, 171);
  public static final CustomColor kElSalvadorGreen = new CustomColor(0, 153, 0);
  public static final CustomColor kGuatemalaBlue = new CustomColor(73, 151, 208);
  public static final CustomColor kGuatemalaGreen = new CustomColor(38, 148, 1);
  public static final CustomColor kMexicoGreen = new CustomColor(0, 99, 65);
  public static final CustomColor kMexicoBrown = new CustomColor(117, 49, 0);
  public static final CustomColor kMexicoRed = new CustomColor(200, 16, 46);
  public static final CustomColor kCubaBlue = new CustomColor(0, 75, 135);
  public static final CustomColor kRepublicaDominicanaBlue = new CustomColor(0, 45, 98);
  public static final CustomColor kRepublicaDominicanaRed = new CustomColor(206, 17, 38);
  public static final CustomColor kRepublicaDominicanaGreen = new CustomColor(0, 99, 0);
  public static final CustomColor kJamaicaGreen = new CustomColor(0, 155, 58);
  public static final CustomColor kHaitiBlue = new CustomColor(0, 32, 159);
  public static final CustomColor kHaitiRed = new CustomColor(210, 16, 52);
  public static final CustomColor kUnitedStatesBlue = new CustomColor(10, 49, 97);
  public static final CustomColor kUnitedStatesRed = new CustomColor(179, 25, 66);
  public static final CustomColor kCanadaRed = new CustomColor(216, 6, 33);
  public static final CustomColor kPuertoRicoRed = new CustomColor(206, 0, 0);
  public static final CustomColor kPuertoRicoBlue = new CustomColor(51, 173, 255);
}