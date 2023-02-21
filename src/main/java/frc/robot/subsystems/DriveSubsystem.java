package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import java.util.function.DoubleSupplier;


//Set up all the system objects (motors and encoders) 
//note, constants need to be updated as of Feb 20
public class DriveSubsystem extends SubsystemBase {
  private final VictorSP MotorLeft1 = new VictorSP(DriveConstants.MOTORLEFT1PORT);
  private final VictorSP MotorLeft2 = new VictorSP(DriveConstants.MOTORLEFT2PORT);  
  private final VictorSP MotorRight1 = new VictorSP(DriveConstants.MOTORRIGHT1PORT);
  private final VictorSP MotorRight2 = new VictorSP(DriveConstants.MOTORRIGHT2PORT);


// The motors on the right side of the drive.
private final MotorControllerGroup ControlGroupRightMotors =
  new MotorControllerGroup(MotorRight1,MotorRight2);

// The motors on the left side of the drive
private final MotorControllerGroup ControlGroupLeftMotors =
  new MotorControllerGroup(MotorLeft1,MotorLeft2);

  // The left-side drive encoder
  private final Encoder EncoderLeft =
      new Encoder(
          DriveConstants.ENCODERPORTSLEFT[0],
          DriveConstants.ENCODERPORTSLEFT[1],
          DriveConstants.ENCODERREVERSEDLEFT);

  // The right-side drive encoder
  private final Encoder EncoderRight =
      new Encoder(
          DriveConstants.ENCODERPORTSRIGHT[0],
          DriveConstants.ENCODERPORTSRIGHT[1],
          DriveConstants.ENCODERREVERSEDRIGHT);

  private final DifferentialDrive DiffDrive = new DifferentialDrive(ControlGroupLeftMotors,ControlGroupRightMotors);
  
  
  public void initialize(){
    //ControlGroupRightMotors.setInverted(true); //must decide if we want to keep this, or switch the wires so that the lights 
    //on the victor spx motro controllers all flas green when driving forward.

    // Sets the distance per pulse for the encoders
    EncoderLeft.setDistancePerPulse(DriveConstants.ENCODERDISTANCEPERPULSEINCHES);
    EncoderRight.setDistancePerPulse(DriveConstants.ENCODERDISTANCEPERPULSEINCHES);
  }
  // simple arcade drive with squared inputs
  public void arcadeDriveSquared(Double fwd, Double rot){

    DiffDrive.arcadeDrive(Math.abs(fwd)*fwd,Math.abs(rot)*rot);
  }

  // Untested, this should should reduce the sensitivity of the steering 
  //based on the real speed of the tracks as read by the encoders
  public void arcadeDriveAdaptiveSteering(Double fwd, Double rot){
    double adaptedrot= rot/(1+((EncoderLeft.getRate()+EncoderRight.getRate())*DriveConstants.ADAPTIVESTEERINGSENSITIVITY));
    DiffDrive.arcadeDrive(Math.abs(fwd)*fwd, adaptedrot);
  }
  

  
  // Untested, this should use PID to make it drive straight
  //it probably will have some issues as some of the values approach zero
  PIDController drivePID = new PIDController(0, 0, 0);//currently set to zero just to test how it reacts without PID
  public void driveStraightPID(Double fwd, Double rot){
    double adaptedrot= rot/(1+((EncoderLeft.getRate()+EncoderRight.getRate())*DriveConstants.ADAPTIVESTEERINGSENSITIVITY));
    double rightSpeedTarget= DifferentialDrive.arcadeDriveIK(fwd, adaptedrot, false).right;
    double leftSpeedTarget =DifferentialDrive.arcadeDriveIK(fwd, adaptedrot, false).left;;//no rotation
    double relativeLeft= (leftSpeedTarget!=0)?EncoderLeft.getRate()/leftSpeedTarget:0;
    double relativeRight= (rightSpeedTarget!=0)?EncoderRight.getRate()/rightSpeedTarget:0;//EncoderRight.getRate()/fwd;
    double PIDCorrection= drivePID.calculate((relativeLeft-relativeRight),0);

    DiffDrive.tankDrive(leftSpeedTarget+PIDCorrection, rightSpeedTarget-PIDCorrection);
  }

  
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

/**
   * Returns a command that drives the robot with arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
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
}
