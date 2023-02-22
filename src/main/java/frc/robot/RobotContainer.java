package frc.robot;

import frc.robot.Constants.ButtonMappings;
import frc.robot.commands.FollowTargets;
import frc.robot.commands.RunMotor;
import frc.robot.commands.StopMotor;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.PS4Controller;
import frc.robot.subsystems.LimelightSubsystem;

public class RobotContainer {
  private final DriveSubsystem DriveSubsystem = new DriveSubsystem();
  private final LimelightSubsystem LimelightSubsystem = new LimelightSubsystem();

  PS4Controller ps4 = new PS4Controller(Constants.Driver_Controller);
  public RobotContainer() {
    configureBindings();
    DriveSubsystem.initialize();
    LimelightSubsystem.setPipeline(9); //set for Brandons april tag pipeline
  }

  private void configureBindings() {

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
