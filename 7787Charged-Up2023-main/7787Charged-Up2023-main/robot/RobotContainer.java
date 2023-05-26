// Cleaned
package frc.robot;

import java.util.prefs.Preferences;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.ButtonMappings;
import frc.robot.commands.auto.*;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
  
  // Define Subsystems
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final LimelightSubsystem LimelightSubsystem = new LimelightSubsystem();
  private final ArmSubsystem ArmSubsystem = new ArmSubsystem();
  private final NavXSubsystem navXSubsystem = new NavXSubsystem();
  
  // This seems important?
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  // Define controller
  private final XboxController x_box_taiga = new XboxController(Constants.DRIVER_CONTROLLER_TAIGA);

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
    
        
  }

  // Initilizes subsystems
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

  // Configure button bindings
  private void configureBindings() {
  
    new JoystickButton(this.x_box_taiga, 5).onTrue(this.driveSubsystem.shiftHigh());
    new JoystickButton(this.x_box_taiga, 6).onTrue(this.driveSubsystem.shiftLow());
    new JoystickButton(this.x_box_taiga, 7).onTrue(this.ArmSubsystem.setArmPositionCommand(120, 11,0)); // Start home test limit.
    new JoystickButton(this.x_box_taiga, 1).onTrue(this.ArmSubsystem.setArmPositionCommand(75, 30)); // Circle Ground
    new JoystickButton(this.x_box_taiga, 2).onTrue(this.ArmSubsystem.setArmPositionCommand(90, 79));  // Square Medium
    new JoystickButton(this.x_box_taiga, 3).onTrue(this.ArmSubsystem.setArmPositionCommand(66, 123)); // X High
    new JoystickButton(this.x_box_taiga, 4).onTrue(this.ArmSubsystem.setArmPositionCommand(112, 68)); // 112, 63 Match drive substation
    new JoystickButton(this.ps4_bri, ButtonMappings.CIRCLE).onTrue(this.ArmSubsystem.setArmAzimuthCommand(-90)); // Circle Right -90
    
  }
  public void configureDefaultCommands() {

    // Limelight Subsytem default commands
    this.LimelightSubsystem.setDefaultCommand(
      this.LimelightSubsystem.checkForTargetsCommand()
    );

    // Drive Subsytem default commands
    this.driveSubsystem.setDefaultCommand(
          this.driveSubsystem.arcadeDriveSquaredCommand(
              () -> -this.x_box_taiga.getLeftY(), 
              () -> -this.x_box_taiga.getLeftX()
            )

     );

    this.ArmSubsystem.setDefaultCommand(
         this.ArmSubsystem.analogArmInputsCommand(
             () -> (this.x_box_taiga.getRightX() * -10),
             () -> ((this.x_box_taiga.getRightY() * -10) + ((this.x_box_taiga.getLeftTriggerAxis() + 0) * 10)),
             () -> (this.x_box_taiga.getRightY() * -15 + ((this.x_box_taiga.getLeftTriggerAxis() + 0) * 20)),
             () -> ((this.x_box_taiga.getRightTriggerAxis() + 0) * 44)
         )
    
    );

  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
