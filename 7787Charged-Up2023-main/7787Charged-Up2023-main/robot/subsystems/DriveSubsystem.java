package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import java.util.function.DoubleSupplier;


//Set up all the system objects (motors and encoders) 
//note, constants need to be updated as of Feb 20
public class DriveSubsystem extends SubsystemBase {

  NavXSubsystem navX;

  private final VictorSP MotorLeft1 = new VictorSP(DriveConstants.MOTOR_LEFT1_PORT);
  private final VictorSP MotorLeft2 = new VictorSP(DriveConstants.MOTOR_LEFT2_PORT);  
  private final VictorSP MotorRight1 = new VictorSP(DriveConstants.MOTOR_RIGHT1_PORT);
  private final VictorSP MotorRight2 = new VictorSP(DriveConstants.MOTOR_RIGHT2_PORT);

  private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  private final DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);

  // The motors on the right side of the drive.
  private final MotorControllerGroup ControlGroupRightMotors =
  new MotorControllerGroup(MotorRight1,MotorRight2);

  // The motors on the left side of the drive
  private final MotorControllerGroup ControlGroupLeftMotors =
  new MotorControllerGroup(MotorLeft1,MotorLeft2);

  // The left-side drive encoder
  private final Encoder EncoderLeft =
      new Encoder(
          DriveConstants.ENCODER_PORTS_LEFT[0],
          DriveConstants.ENCODER_PORTS_LEFT[1],
          DriveConstants.ENCODER_REVERSED_LEFT
          );

  // The right-side drive encoder
  private final Encoder EncoderRight =
      new Encoder(
          DriveConstants.ENCODER_PORTS_RIGHT[0],
          DriveConstants.ENCODER_PORTS_RIGHT[1],
          DriveConstants.ENCODER_REVERSED_RIGHT
          );

  private final DifferentialDrive DiffDrive = new DifferentialDrive(ControlGroupLeftMotors,ControlGroupRightMotors);
  
  public DriveSubsystem() {

    // Sets the distance per pulse for the encoders
    //compressor.disable();
    EncoderLeft.setDistancePerPulse(DriveConstants.ENCODER_DISTANCE_PER_PULSE_INCHES);
    EncoderRight.setDistancePerPulse(DriveConstants.ENCODER_DISTANCE_PER_PULSE_INCHES);
    
  }
  public double getLeftEncoderDistance(){
    double encoderDistance = EncoderLeft.getDistance();
    SmartDashboard.putNumber("leftEncoder Distance", encoderDistance);
    return encoderDistance;
  }
  public double getRightEncoderDistance(){
    return EncoderRight.getDistance();
  }

  // simple arcade drive with squared inputs
  public void arcadeDriveSquared(Double fwd, Double rot) {
    SmartDashboard.putNumber("encoder left", EncoderLeft.get());

    DiffDrive.arcadeDrive(Math.abs(fwd) * fwd,Math.abs(rot) * rot);

  }
  public void arcadeDrive(Double fwd, Double rot) {
    SmartDashboard.putNumber("encoder left", EncoderLeft.get());

    DiffDrive.arcadeDrive(fwd, rot);

  }

  // Untested, this should should reduce the sensitivity of the steering 
  //based on the real speed of the tracks as read by the encoders
  public void arcadeDriveAdaptiveSteering(Double fwd, Double rot) {

    double adaptedrot= rot / (1 + ((EncoderLeft.getRate() + EncoderRight.getRate()) * DriveConstants.ADAPTIVE_STEERING_SENSITIVITY));
    DiffDrive.arcadeDrive(Math.abs(fwd) * fwd, adaptedrot);

  }
  
  // Untested, this should use PID to make it drive straight
  //it probably will have some issues as some of the values approach zero
  PIDController drivePID = new PIDController(0, 0, 0);//currently set to zero just to test how it reacts without PID
  PIDController drivePositionLeftPID = new PIDController(1, 0.0, 0.010);//currently set 1 for no ID feedback //Feb28
  PIDController drivePositionRightPID = new PIDController(1, 0.00, 0.010);//currently set 1 for no ID feedback //Feb28

  public CommandBase driveToPositionCommand(double m_fwdInches){  // **** Changed from void to Command for Testing
    //double adaptedrot= rot/(1+((EncoderLeft.getRate()+EncoderRight.getRate())*DriveConstants.ADAPTIVE_STEERING_SENSITIVITY));
    int m_LeftOffset=EncoderLeft.get();
    int m_RightOffset=EncoderRight.get();

    double m_leftPositionTarget;
    double m_rightPositionTarget;
  
    m_rightPositionTarget = m_fwdInches/DriveConstants.ENCODER_ROTATION_PER_PULSE_DEGREES  + m_RightOffset;
    m_leftPositionTarget = m_fwdInches/DriveConstants.ENCODER_ROTATION_PER_PULSE_DEGREES + m_LeftOffset;

    //DiffDrive.tankDrive(m_PIDLeftValue, m_PIDRightValue); **** changed to command based form for testing
    return run( ()->{driveToEncoderPosition(m_leftPositionTarget,m_rightPositionTarget);});
  }

  public void balanceRobot() {

    if (Math.abs(navX.navXPitch()) > 5) {

      while(navX.navXPitch()>9) {
        driveToPositionCommand(1);
      }

      while(navX.navXPitch()<-9) {
        driveToPositionCommand(-1);
      }

    }
    else {
      arcadeDriveAdaptiveSteering(0.0,0.0);
    } 
  }



  public void driveToEncoderPosition(Double m_leftEncoderTarget,double m_rightEncoderTarget){  // **** Changed from void to Command for Testing
    
    double m_PIDLeftValue= drivePositionLeftPID.calculate(EncoderLeft.get(),m_leftEncoderTarget);
    double m_PIDRightValue= drivePositionRightPID.calculate(EncoderRight.get(),m_rightEncoderTarget);

    //DiffDrive.tankDrive(m_PIDLeftValue, m_PIDRightValue); **** changed to command based form for testing
    DiffDrive.tankDrive(m_PIDLeftValue, m_PIDRightValue);
  }

  public CommandBase spinToPosition(Double m_turnDegrees) {   // **** Changed from void to Command for Testing
    int m_LeftOffset=EncoderLeft.get();
    int m_RightOffset=EncoderRight.get();

    double m_leftPositionTarget = 0.0;
    double m_rightPositionTarget = 0.0;
    if (solenoid.get() == Value.kReverse) {
      m_rightPositionTarget = -m_turnDegrees/DriveConstants.ENCODER_ROTATION_PER_PULSE_DEGREES*DriveConstants.HIGH_GEAR_RATIO+m_RightOffset;
      m_leftPositionTarget = m_turnDegrees/DriveConstants.ENCODER_ROTATION_PER_PULSE_DEGREES*DriveConstants.HIGH_GEAR_RATIO+m_LeftOffset;
    }
    else if(solenoid.get() == Value.kForward) {
      m_rightPositionTarget = -m_turnDegrees/DriveConstants.ENCODER_ROTATION_PER_PULSE_DEGREES*DriveConstants.LOW_GEAR_RATIO+m_RightOffset;
      m_leftPositionTarget = m_turnDegrees/DriveConstants.ENCODER_ROTATION_PER_PULSE_DEGREES*DriveConstants.LOW_GEAR_RATIO+m_LeftOffset;
    }


    // double m_rightPositionTarget=-m_turnDegrees/DriveConstants.ENCODER_ROTATION_PER_PULSE_DEGREES+m_RightOffset;  // Changed to if stamement for gear ratio changes
    // double m_leftPositionTarget =m_turnDegrees/DriveConstants.ENCODER_ROTATION_PER_PULSE_DEGREES+m_LeftOffset;
    double m_PIDLeftValue= drivePositionLeftPID.calculate(EncoderLeft.get(),m_leftPositionTarget);
    double m_PIDRightValue= drivePositionRightPID.calculate(EncoderRight.get(),m_rightPositionTarget);

    //DiffDrive.tankDrive(m_PIDLeftValue, m_PIDRightValue); **** changed to command based form for testing
    return runOnce ( ()->DiffDrive.tankDrive(m_PIDLeftValue, m_PIDRightValue));

  }
 
  public void driveStraightPID(Double fwd, Double rot){
    double adaptedrot= rot/(1+((EncoderLeft.getRate()+EncoderRight.getRate())*DriveConstants.ADAPTIVE_STEERING_SENSITIVITY));
    double rightSpeedTarget= DifferentialDrive.arcadeDriveIK(fwd, adaptedrot, false).right;
    double leftSpeedTarget =DifferentialDrive.arcadeDriveIK(fwd, adaptedrot, false).left;;//no rotation
    double relativeLeft= (leftSpeedTarget!=0)?EncoderLeft.getRate()/leftSpeedTarget:0;
    double relativeRight= (rightSpeedTarget!=0)?EncoderRight.getRate()/rightSpeedTarget:0;//EncoderRight.getRate()/fwd;
    double PIDCorrection= drivePID.calculate((relativeLeft-relativeRight),0);

    DiffDrive.tankDrive(leftSpeedTarget+PIDCorrection, rightSpeedTarget-PIDCorrection);
  }

  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}

  public CommandBase arcadeDriveSquaredCommand(DoubleSupplier fwd, DoubleSupplier rot) {
    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
    return run(() -> this.arcadeDriveSquared(fwd.getAsDouble(), rot.getAsDouble()))
        .withName("arcadeDrive");
  }

  public CommandBase arcadeDriveAdaptiveSteeringCommand(DoubleSupplier fwd, DoubleSupplier rot) {
    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
    return run(() -> this.arcadeDriveAdaptiveSteering(fwd.getAsDouble(), rot.getAsDouble()))
        .withName("arcadeDriveAdaptiveSteering");
  }

  /**
   * Returns a command that drives the robot forward a specified distance at a specified speed.
   *
   * @param distanceMeters The distance to drive forward in meters
   * @param speed The fraction of max speed at which to drive
   */
  public CommandBase driveDistanceCommand(double distanceMeters, double speed) {
    return runOnce(
            () -> {
              // Reset encoders at the start of the command
              EncoderLeft.reset();
              EncoderRight.reset();
            })
        // Drive forward at specified speed
        .andThen(run(() -> DiffDrive.arcadeDrive(speed, 0)))
        // End command when we've traveled the specified distance
        .until(
            () ->
                Math.max(EncoderLeft.getDistance(), EncoderRight.getDistance())
                    >= distanceMeters)
        // Stop the drive when the command ends
        .finallyDo(interrupted -> DiffDrive.stopMotor());
  }
  public CommandBase shiftLow() {
    return runOnce( ()-> solenoid.set(Value.kForward));

  }
  public CommandBase shiftHigh() {
    return runOnce( ()-> solenoid.set(Value.kReverse));
  
  }
  public CommandBase compressorOn() {
    return runOnce( ()->compressor.enableDigital());
  
  }
  public CommandBase compressorOff() {
    return runOnce( ()->compressor.disable());
  
  }
}