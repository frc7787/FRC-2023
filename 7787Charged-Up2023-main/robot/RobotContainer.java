package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

import frc.robot.Constants.ButtonMappings;
import frc.robot.commands.auto.*;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.prefs.Preferences;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
  
  // Defines the Arm, Drive, and Limelight subsystems.
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final LimelightSubsystem LimelightSubsystem = new LimelightSubsystem();
  private final ArmSubsystem ArmSubsystem = new ArmSubsystem();
  private final NavXSubsystem navXSubsystem = new NavXSubsystem();
  

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  //private final Auto m_autoCommand = new Auto(DriveSubsystem, ArmSubsystem);
  
  // private final SparkmaxMotor testSparkmaxMotor = new SparkmaxMotor(5); // **** Feb 24
  
  // Creates the PS4Controller
  //private final PS4Controller ps4_taiga = new PS4Controller(Constants.DRIVER_CONTROLLER_TAIGA);
  private final XboxController x_box_taiga = new XboxController(Constants.DRIVER_CONTROLLER_TAIGA);

 // private final PS4Controller ps4_bri = new PS4Controller(Constants.DRIVER_CONTROLLER_BRI);

  private void initializeAutoChooser() {
    m_autoChooser.setDefaultOption("Do one second", new RunMotorTimed(this.driveSubsystem,1));
    m_autoChooser.addOption("run motor 3s", new RunMotorTimed(this.driveSubsystem,3));
    m_autoChooser.addOption("Sequential 4 seconds", new DriveAndParkBrandon(this.driveSubsystem));
    m_autoChooser.addOption("drive120", new DriveTenFeet(this.driveSubsystem));

    SmartDashboard.putData("Auto Selector", m_autoChooser);
  }
  
  // Defines robot container
  public RobotContainer() {

    configureBindings();
    SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
    initializeAutoChooser();

    // Configure the trigger bindings
    
        
  }

  // Initilizes all of the subsystems
  public LimelightSubsystem getLimelightSubsystem(){
    return this.LimelightSubsystem;
  }
  public ArmSubsystem getArmSubsystem(){
    return this.ArmSubsystem;
  }
  public DriveSubsystem getDriveSubsystem(){
    return this.driveSubsystem;
  }
  public NavXSubsystem getNavXSubsystem() {
    return this.navXSubsystem;
  }


  private void configureBindings() {
  
    //xbox controller
    // kLeftX(0),
    // kRightX(4),
    // kLeftY(1),
    // kRightY(5),
    // kLeftTrigger(2),
    // kRightTrigger(3);
    //    kLeftBumper(5),
    // kRightBumper(6),
    // kLeftStick(9),
    // kRightStick(10),
    // kA(1),
    // kB(2),
    // kX(3),
    // kY(4),
    // kBack(7),
    // kStart(8);
    // PS4 bindings
    
    new JoystickButton(this.x_box_taiga, 5).onTrue(this.driveSubsystem.shiftHigh());
    new JoystickButton(this.x_box_taiga, 6).onTrue(this.driveSubsystem.shiftLow());
    //new JoystickButton(this.ps4, ButtonMappings.SQUARE).onTrue(this.DriveSubsystem.compressorOn());
    //new JoystickButton(this.ps4, ButtonMappings.CROSS).onTrue(this.DriveSubsystem.compressorOff());
    //new JoystickButton(this.ps4, ButtonMappings.L1).whileTrue(this.DriveSubsystem.driveToPositionCommand(2.0));
    // new JoystickButton(this.ps4, ButtonMappings.L2).onTrue(this.DriveSubsystem.spinToPosition(360.0));
    new JoystickButton(this.x_box_taiga, 7).onTrue(this.ArmSubsystem.setArmPositionCommand(120, 11,0));// start home test limit
    new JoystickButton(this.x_box_taiga, 1).onTrue(this.ArmSubsystem.setArmPositionCommand(75, 30));// circle ground
    new JoystickButton(this.x_box_taiga, 2).onTrue(this.ArmSubsystem.setArmPositionCommand(90, 79));  //  square Medium
    new JoystickButton(this.x_box_taiga, 3).onTrue(this.ArmSubsystem.setArmPositionCommand(66, 123)); // x High
    new JoystickButton(this.x_box_taiga, 4).onTrue(this.ArmSubsystem.setArmPositionCommand(112, 68));//112,63 match triangle substation

  //   new JoystickButton(this.ps4_bri, ButtonMappings.START).onTrue(this.ArmSubsystem.setArmPositionCommand(120,11,0));// start same home position
  //   new JoystickButton(this.ps4_bri, ButtonMappings.CIRCLE).onTrue(this.ArmSubsystem.setArmAzimuthCommand(-90));// circle right -90
  //   new JoystickButton(this.ps4_bri, ButtonMappings.SQUARE).onTrue(this.ArmSubsystem.setArmAzimuthCommand(90));  //  square left 90
  //  // new JoystickButton(this.ps4_bri, ButtonMappings.CROSS).onTrue(this.ArmSubsystem.setArmAzimuthCommand(???)); // x unassigned
  //   new JoystickButton(this.ps4_bri, ButtonMappings.TRIANGLE).onTrue(this.ArmSubsystem.setArmAzimuthCommand(0));// triangle forward
  //   new JoystickButton(this.ps4_bri, ButtonMappings.R1).onTrue(this.ArmSubsystem.setArmAzimuthR30Command());// triangle forward
  //   new JoystickButton(this.ps4_bri, ButtonMappings.L1).onTrue(this.ArmSubsystem.setArmAzimuthL30Command());// triangle forward
    



    

    
  }
  public void configureDefaultCommands() {

    // Limelight Subsytem default commands
    this.LimelightSubsystem.setDefaultCommand(
      this.LimelightSubsystem.checkForTargetsCommand()
    );

    // Drive Subsytem default commands
    this.driveSubsystem.setDefaultCommand(
          this.driveSubsystem.arcadeDriveSquaredCommand(
            //() -> (0),() -> (0))//**** March 1st disable drive
              () -> -this.x_box_taiga.getLeftY(), () -> -this.x_box_taiga.getLeftX()) // Taiga Drive **** Feb 28 to test encoders for drive motors


     );

    // **** Feb 24
    // this.ArmSubsystem.setDefaultCommand(
    //      this.ArmSubsystem.joystickMotorCommand(
    //          () -> (this.ps4.getRightX()*-30),() -> (this.ps4.getR2Axis()*30+30),() -> (this.ps4.getRightY()*-80),() -> (this.ps4.getL2Axis()*-20))
    // );
    this.ArmSubsystem.setDefaultCommand(
         this.ArmSubsystem.analogArmInputsCommand(//azimuth shoulder elbow claw
              //() -> (0),() -> (0),() -> (0),() -> (this.ps4_taiga.getR2Axis()*30+30))//1 **** March 1st
             //() -> (this.ps4_bri.getRightX()*-30),() -> ((this.ps4_bri.getLeftY()*-30)+((this.ps4_bri.getL2Axis()+1)*10)),() -> (this.ps4_bri.getRightY()*-30+((this.ps4_bri.getL2Axis()+1)*20)),() -> ((this.ps4_bri.getR2Axis()+1)*22))// for fine controll add to other controller
             //() -> (this.ps4_bri.getRightX()*-10),() -> ((this.ps4_bri.getLeftY()*-10)+((this.ps4_bri.getL2Axis()+1)*5)),() -> (this.ps4_bri.getRightY()*-15+((this.ps4_bri.getL2Axis()+1)*10)),() -> ((this.ps4_bri.getR2Axis()+1)*22))// for fine controll add to other controller
             () -> (this.x_box_taiga.getRightX()*-10),() -> ((this.x_box_taiga.getRightY()*-10)+((this.x_box_taiga.getLeftTriggerAxis()+0)*10)),() -> (this.x_box_taiga.getRightY()*-15+((this.x_box_taiga.getLeftTriggerAxis()+0)*20)),() -> ((this.x_box_taiga.getRightTriggerAxis()+0)*44))// for fine controll add to other controller
    
    );

    // New command to follow targets while true.
    //new Trigger(this.LimelightSubsystem::hasTargets).whileTrue( new FollowTargets(this.DriveSubsystem,this.LimelightSubsystem));

    
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
    //return new RunMotor(this.driveSubsystem,5);
  }
}
