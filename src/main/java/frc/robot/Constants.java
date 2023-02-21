// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class OperatorConstants {
    public static final int KDRIVERCONTROLLERPORT = 0;
  }

  public static final class ArmConstants {

    public static final int THETA1MAX = 5;
    public static final int THETA2MAX = 4;
    public static final int THETA1MIN = 6;
    public static final int THETA2MIN = 7;
    public static final int ARMLENGTH1 = 40; // Inches
    public static final int ARMLENGTH2 = ARMLENGTH1;// Note if different lengths math will change

  }

  public static final class DriveConstants {
    public static final int MOTORLEFT1PORT = 0;
    public static final int MOTORLEFT2PORT = 1;
    public static final int MOTORRIGHT1PORT = 2;
    public static final int MOTORRIGHT2PORT = 3;

    public static final int[] ENCODERPORTSLEFT = {0, 1};
    public static final int[] ENCODERPORTSRIGHT = {2, 3};
    public static final boolean ENCODERREVERSEDLEFT = false;
    public static final boolean ENCODERREVERSEDRIGHT= true;

    public static final double ADAPTIVESTEERINGSENSITIVITY = 0.002;

    public static final int ENCODERTICKSPERREV= 1024;
    
    public static final double ENCODERDISTANCEPERPULSEINCHES =3.0;
        
  }
  public static final class ButtonMappings {
    public static final int SQUARE = 0;
    public static final int CROSS = 1;
    public static final int CIRCLE = 2;
    public static final int TRIANGLE = 3;
    public static final int L1 = 4;
    public static final int R1 = 5;
    public static final int L2 = 6;
    public static final int R2 = 7;
    public static final int SHARE = 8;
    public static final int START = 9;
    public static final int L3 = 10;
    public static final int R3 = 11;
    public static final int PS = 12;
    public static final int TOUCHPAD = 13;
  }
  public static final int Driver_Controller = 0;

}
