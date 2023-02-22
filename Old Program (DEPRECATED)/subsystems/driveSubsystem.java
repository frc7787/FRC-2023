package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class driveSubsystem extends SubsystemBase {
    // Constants
    private static final int rightDeviceID = 2;
    private static final int leftDeviceID = 3;
    private CANSparkMax m_leftMotor = new CANSparkMax(leftDeviceID, MotorType.kBrushless) ;
    private CANSparkMax m_rightMotor = new CANSparkMax(rightDeviceID, MotorType.kBrushless);


  /** Creates a new ExampleSubsystem. */
  public driveSubsystem() {

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          LimeLightSubsystem.setPipeline(7);
        });
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
    m_leftMotor.set(1);
    m_rightMotor.set(1);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


  public void setMotorSpeed(float speed) {
    m_leftMotor.set(speed);
    m_rightMotor.set(speed);

}

}
