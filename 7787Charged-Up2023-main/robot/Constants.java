package frc.robot;

public final class Constants {

  public static final class OperatorConstants {

    // Drive Controll Port
    //public static final int K_DRIVER_CONTROLLER_PORT = 0;
    
  }

  // Constants for Arm Subsystem
  public static final class ArmConstants {

    // Arm angle mininum values in degrees
    public static final int THETA1_MAX = 135;
    public static final int THETA2_MAX = 180;
    public static final int THETA1_MIN = 30;
    public static final int THETA2_MIN = 30;

    public static final int THETA1_START = 98; //ANGLE FROM BASE BACK TO SEGMENT 1
    public static final int THETA2_START = 10;//ANGLE FROM ARM SEGMENT  1 TO 2
    public static final int AZIMUTH_START = -9;//for claw open positon

  

    // Arm Lengths
    public static final int ARM_LENGTH1 = 40; // Arm Length is in inches
    public static final int ARM_LENGTH2 = ARM_LENGTH1; 

    // Motor ID's
    public static final int AZIMUTH_MOTOR_ID = 2;
    public static final int SHOULDER_MOTOR_ID = 3;
    public static final int ELBOW_MOTOR_ID = 4;
    public static final int CLAW_MOTOR_ID = 5;

    //Resseting Constants
    public static final Double RESETTING_SPEED = 0.1;

    // Limit Switch Directions ; True = Forward, False = Reverse
    public static final boolean AZIMUTH_LIMIT_SWITH_DIRECTION = false;
    public static final boolean SHOULDER_LIMIT_SWITCH_DIRECTION = false;
    public static final boolean ELBOW_LIMIT_SWITCH_DIRECTION = false;
    public static final boolean CLAW_LIMIT_SWITCH_DIRECTION = false;

    // degree conversions
    public static final double AZIMUTH_DEGREES_PER_REVOLUTION= 152.3/90;
    public static final double SHOULDER_DEGREES_PER_REVOLUTION= (22.9-6)/20;
    public static final double ELBOW_DEGREES_PER_REVOLUTION = 153.0/100;
     //1;(44-16)/20;
    public static final double CLAW_DEGREES_PER_REVOLUTION= 1;





    // Limit Offsets
    public static final int AZIMUTH_OFFSET = -15;
    public static final int SHOULDER_OFFSET = 130;
    public static final int ELBOW_OFFSET = 10;

  }

  // Constants for drive subsystem
  public static final class DriveConstants {
    
    // Motor Ports
    public static final int MOTOR_LEFT1_PORT = 0;
    public static final int MOTOR_LEFT2_PORT = 1;
    public static final int MOTOR_RIGHT1_PORT = 2;
    public static final int MOTOR_RIGHT2_PORT = 3;

    // Encoder Ports
    public static final int[] ENCODER_PORTS_LEFT = {0, 1};
    public static final int[] ENCODER_PORTS_RIGHT = {2, 3};

    // Encoder Directions
    public static final boolean ENCODER_REVERSED_LEFT = true;
    public static final boolean ENCODER_REVERSED_RIGHT= true;

    // Adaptive Steering Sensitivity
    public static final double ADAPTIVE_STEERING_SENSITIVITY = 0.002;

    // Encoder Ticks Per Revolution
    public static final int ENCODER_TICKS_PER_REV= 1024;
    
    // Encoder Distance
    public static final double ENCODER_DISTANCE_PER_PULSE_INCHES =0.041;//0.041 feb 28th

    // Encoder Ticks Per Degree
    public static final double ENCODER_ROTATION_PER_PULSE_DEGREES = 0.255;// 0.255 feb 28
    // wheel base 18.42" = 1411 ticks per 360

    // Gear Ratios
    public static final double LOW_GEAR_RATIO = 4;
    public static final double HIGH_GEAR_RATIO = 1;
        
  }
  public static final class ButtonMappings {

    // Button Mappings
    public static final int SQUARE = 0+1;
    public static final int CROSS = 1+1;
    public static final int CIRCLE = 2+1;
    public static final int TRIANGLE = 3+1;

    // Trigger Mappings
    public static final int L1 = 4+1;
    public static final int R1 = 5+1;
    public static final int L2 = 6+1;
    public static final int R2 = 7+1;

    // Function Button Mappings
    public static final int SHARE = 8+1;
    public static final int START = 9+1;

    // Joystick Mappings
    public static final int L3 = 10+1;
    public static final int R3 = 11+1;

    // Playstation Button Maping
    public static final int PS = 12+1;

    // Touch Pad Mappings
    public static final int TOUCH_PAD = 13+1;
  }

  // Driver Controller
  public static final int DRIVER_CONTROLLER_TAIGA = 0;

  public static final int DRIVER_CONTROLLER_BRI = 1;
}