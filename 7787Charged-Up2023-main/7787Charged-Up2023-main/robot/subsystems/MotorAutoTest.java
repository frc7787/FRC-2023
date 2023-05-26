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
public class MotorAutoTest extends SubsystemBase {

  NavXSubsystem navX;

  private final VictorSP Motor1 = new VictorSP(5);

  // The motors on the right side of the drive.
  

  // The motors on the left side of the drive
  





  
  
  public MotorAutoTest() {

    // Sets the distance per pulse for the encoders
    //compressor.disable();
    
  }

  // simple arcade drive with squared inputs
  

  // Untested, this should should reduce the sensitivity of the steering 
  //based on the real speed of the tracks as read by the encoders
  
  
  // Untested, this should use PID to make it drive straight
  //it probably will have some issues as some of the values approach zero
  
 
  

  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {}

  public void pwrSet(double power) {
    Motor1.set(power);
  }

  @Override
  public void simulationPeriodic() {}

  

  

  /**
   * Returns a command that drives the robot forward a specified distance at a specified speed.
   *
   * @param distanceMeters The distance to drive forward in meters
   * @param speed The fraction of max speed at which to drive
   */
  
  
}