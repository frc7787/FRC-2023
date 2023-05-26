package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import java.util.function.DoubleSupplier;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SparkmaxMotor extends SubsystemBase {
  
  private CANSparkMax m_motor;
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private boolean isZeroed=false;
  private boolean limitSwitchDirection=false; // False reverse ; True forward
  private SparkMaxLimitSwitch limitSwitch;

  public SparkmaxMotor(int deviceID,Boolean limitSwitchDirection) {
    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_pidController = m_motor.getPIDController();
    m_encoder = m_motor.getEncoder();
    limitSwitch = m_motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    
    // PID coefficients
    kP = 0.1; 
    kI = 0; //1e-4;
    kD = 0.01; //was 1 feb 24 test
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 0.2; //1; feb 24 test
    kMinOutput = -0.2; //-1; feb 24 test

    // Set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
  }

  public RelativeEncoder getEncoder(){
    return this.m_encoder;
  }

  public boolean zeroEncoder(double targetVelocity){
    if (!isZeroed && !limitSwitch.isPressed()) {
     if (limitSwitchDirection) {
      // Set velocity forward
      m_pidController.setReference(ArmConstants.RESETTING_SPEED, CANSparkMax.ControlType.kVelocity);
     } else {
      // Set velocity reverse
      m_pidController.setReference(-ArmConstants.RESETTING_SPEED, CANSparkMax.ControlType.kVelocity);

     }
    } else{
      m_pidController.setReference(0, CANSparkMax.ControlType.kVelocity);
      m_encoder.setPosition(0);
      isZeroed=true;
    } 
    return isZeroed;
  }

  public void runToposition(double targetPosition){
   
    m_pidController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
  }
  public void runTopositionAndEncoder(double targetPosition){
   
    m_pidController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
  
  }

  public boolean exampleCondition() {
    // Query boolean sate
    return false;
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}


  public CommandBase joystickMotorCommand(DoubleSupplier Yval) {
    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
    return run(() -> this.runTopositionAndEncoder(Yval.getAsDouble()))
        .withName("Joystick Motor");
  }
}