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
    public static final int K_DRIVER_CONTROLLER_PORT = 0;
  }

  public static final class ArmConstants {

    public static final int THETA1_MAX = 5;
    public static final int THETA2_MAX = 4;
    public static final int THETA1_MIN = 6;
    public static final int THETA2_MIN = 7;
    public static final int ARM_LENGTH1_INCHES = 40; 
    public static final int ARMLENGTH2 = ARMLENGTH1; // Note if different lengths math will change

  }

  public static final class DriveConstants {
    public static final int MOTOR_LEFT1_PORT = 0;
    public static final int MOTOR_LEFT2_PORT = 1;
    public static final int MOTOR_RIGHT1_PORT = 2;
    public static final int MOTOR_RIGHT2_PORT = 3;

    public static final int[] ENCODER_PORTS_LEFT = {0, 1};
    public static final int[] ENCODER_PORTS_RIGHT = {2, 3};
    public static final boolean ENCODER_REVERSED_LEFT = false;
    public static final boolean ENCODER_REVERSED_RIGHT= true;

    public static final double ADAPTIVE_STEERING_SENSITIVITY = 0.002;

    public static final int ENCODER_TICKS_PER_REV= 1024;
    
    public static final double ENCODER_DISTANCE_PER_PULSE_INCHES =3.0;
        
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
  public static final int DRIVER_CONTROLLER = 0;

}
