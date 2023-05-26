package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import java.util.function.DoubleSupplier;

//import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// // **** feb 24
// import edu.wpi.first.wpilibj.Timer;

public class SparkmaxMotor extends SubsystemBase {
  
  private CANSparkMax m_motor;
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private boolean isZeroed=false;
  private boolean limitSwitchDirection=false; //false reverse true forward
  private SparkMaxLimitSwitch limitSwitch;

  // **** feb 24 
  // private double position = 0;
  // private Timer Timer1;

  public SparkmaxMotor(int deviceID,Boolean limitSwitchDirection) {
    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_pidController = m_motor.getPIDController();
    m_encoder = m_motor.getEncoder();
    limitSwitch = m_motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    
    
    
    
    
    // PID coefficients
    kP = 0.1; 
    kI = 0;//1e-4;
    kD = 0.01; //was 1 feb 24 test
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 0.2;//1; feb 24 test
    kMinOutput = -0.2;//-1; feb 24 test

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    //on creation of the motor
    // **** feb 24
    //Timer1 = new Timer();//
    //Timer1.start();//
  }

  public SparkmaxMotor(int deviceID,Boolean limitSwitchDirection, double l_max_output, double l_min_output) {
    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_pidController = m_motor.getPIDController();
    m_encoder = m_motor.getEncoder();
    limitSwitch = m_motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    
    
    
    
    
    // PID coefficients
    kP = 0.1; 
    kI = 0;//1e-4;
    kD = 0.01; //was 1 feb 24 test
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = l_max_output;//1; march 2 test
    kMinOutput = -l_min_output;//-1; march 2 test

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    //on creation of the motor
    // **** feb 24
    //Timer1 = new Timer();//
    //Timer1.start();//
  }

  // private void runToTimer(){
  //   position =  Timer1.get();//
   
  //   m_pidController.setReference(position, CANSparkMax.ControlType.kPosition);
  // }
  // public void startTimer(){
  //   Timer1.start();//
  // }
  // public double getEncoderValue(){
  //   return this.m_encoder.getPosition();//
  // }

  public RelativeEncoder getEncoder(){
    return this.m_encoder;
  }
  public boolean zeroEncoder(double targetVelocity){
    if (!isZeroed&& !limitSwitch.isPressed()) {
     if (limitSwitchDirection) {
      //set velocity forward
      m_pidController.setReference(ArmConstants.RESETTING_SPEED, CANSparkMax.ControlType.kVelocity);
     }
     else {
      //set velocity reverse
      m_pidController.setReference(-ArmConstants.RESETTING_SPEED, CANSparkMax.ControlType.kVelocity);

     }
    }
    else{
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
    //LimelightSubsystem.setPipeline((int)this.m_encoder.getPosition());
  }

  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {}

   // This method will be called once per scheduler run during simulation
  @Override
  public void simulationPeriodic() {}


  //Commands
  public CommandBase joystickMotorCommand(DoubleSupplier Yval) {
    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
    return run(() -> this.runTopositionAndEncoder(Yval.getAsDouble()))
        .withName("Joystick Motor");
  }
}