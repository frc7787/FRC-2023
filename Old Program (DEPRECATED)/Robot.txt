package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.I2C.Port;
//import edu.wpi.first.wpilibj.PS4Controller.Button;
//import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax; 
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.LimeLightSubsystem;

//import edu.wpi.first.wpilibj.SpeedControllerGroup;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {
  
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  private final VictorSP leftMotor1 = new VictorSP(0);//front left
  private final VictorSP leftMotor2 = new VictorSP(2);  
  private final VictorSP rightMotor1 = new VictorSP(1);//front right
  private final VictorSP rightMotor2 = new VictorSP(3);
  //private static final int rightDeviceID = 2;
  private static final int leftDeviceID = 3;
  //private CANSparkMax m_leftMotor = new CANSparkMax(leftDeviceID, MotorType.kBrushless) ;
  //private CANSparkMax m_rightMotor = new CANSparkMax(rightDeviceID, MotorType.kBrushless);
  
  //DifferentialDrive driveFront = new DifferentialDrive(leftMotor1, rightMotor1);
  //DifferentialDrive driveRear = new DifferentialDrive(leftMotor2, rightMotor2);
  //Joystick joystick = new Joystick(0);
  LimeLightSubsystem LimeLight = new LimeLightSubsystem();
  
  //DifferentialDrive m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);
  @Override

  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    LimeLight.setPipeline(9);//set for brandons april tag pipeline
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    LimeLight.getTargets();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //double speed1 = joystick.getY()*Math.abs(joystick.getY())*.9;
    //double speedR = joystick.getX()*Math.abs(joystick.getX())*.8;
    //driveRear.arcadeDrive(speed1, speedR);
    //driveFront.arcadeDrive(speed1,speedR);
    //m_myRobot.arcadeDrive(speed1, speedR);
  }
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    //double speed1 = joystick.getY()*.9;
    //double speedR = (joystick.getX()*.8)/(Math.abs(speed1)*2+1);
    //driveRear.arcadeDrive(speed1, speedR);
    //driveFront.arcadeDrive(speed1,speedR);
    //m_myRobot.arcadeDrive(speed1, speedR);
  }
  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
