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

public class MotorAutoTest extends SubsystemBase {

  NavXSubsystem navX;

  private final VictorSP Motor1 = new VictorSP(5);

  public boolean exampleCondition() {
    // Query boolean state
    return false;
  }

  @Override
  public void periodic() {}

  public void pwrSet(double power) {
    Motor1.set(power);
  }

  @Override
  public void simulationPeriodic() {}
  
}