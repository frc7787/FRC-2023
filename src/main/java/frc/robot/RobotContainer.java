// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ButtonMappings;
import frc.robot.commands.FollowTargets;
// import frc.robot.Constants.OperatorConstants;
// import frc.robot.commands.Autos;
// import frc.robot.commands.ExampleCommand;
import frc.robot.commands.RunMotor;
import frc.robot.commands.StopMotor;
import frc.robot.subsystems.*;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.PS4Controller;
import frc.robot.subsystems.LimelightSubsystem;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveSubsystem DriveSubsystem = new DriveSubsystem();
  private final LimelightSubsystem LimelightSubsystem = new LimelightSubsystem();
  //private final ArmSubsystem ArmSystem = new ArmSubsystem();


  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final CommandPS4Controller m_driverController =
  //     new CommandPS4Controller(OperatorConstants.KDRIVERCONTROLLERPORT);

  PS4Controller ps4 = new PS4Controller(Constants.Driver_Controller);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriveSubsystem.initialize();
    LimelightSubsystem.setPipeline(9);//set for brandons april tag pipeline
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    // PS4 bindings
    new JoystickButton(ps4, ButtonMappings.R1).whileTrue(new RunMotor(DriveSubsystem));//changed to whileTrue
    new JoystickButton(ps4, ButtonMappings.R1).whileFalse(new StopMotor(DriveSubsystem));//changed to whileFalse
    DriveSubsystem.setDefaultCommand(
          DriveSubsystem.arcadeDriveSquaredCommand(
              () -> -ps4.getLeftY(), () -> -ps4.getLeftX())
    );
    LimelightSubsystem.setDefaultCommand(
      LimelightSubsystem.checkForTargetsCommand()
    );

    //Trigger bindings (events)
    //new Trigger(LimeLight::hasTargets).onTrue(LimeLight.getTargetsCommand());
    new Trigger(LimelightSubsystem::hasTargets).whileTrue(new FollowTargets(DriveSubsystem,LimelightSubsystem));

    
  }
// 
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
  //}
}
