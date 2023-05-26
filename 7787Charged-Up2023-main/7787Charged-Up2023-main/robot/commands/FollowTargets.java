package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

// Follows Target using the limelight
public class FollowTargets extends CommandBase {

    // Suppres warnings in the code, because we couldn't be bothered to fix them
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    // Creates an instance of the Drive, and Limelight subsystem
    private DriveSubsystem m_DriveSubsystem;
    private LimelightSubsystem m_LimelightSubsystem;

    // Create Depenencies for the Command.
    public FollowTargets(DriveSubsystem subsystem_drive, LimelightSubsystem subsystem_limelight) {

       m_DriveSubsystem = subsystem_drive;
       m_LimelightSubsystem= subsystem_limelight;

       addRequirements(subsystem_drive);
    }
  
    // Called when the command is initialized
    @Override
    public void initialize() {}
    
    // Called every time the scheduler runs.
    @Override
    public void execute() {
      
      // Gets the targat position
      double targetPosition = m_LimelightSubsystem.getTargets()[0];
      
      // Rotates towards the target depending on where the target is
      double rot = Math.min(Math.max((targetPosition/Math.abs(targetPosition))*Math.sqrt(Math.abs(targetPosition/20)),-0.8),0.8);
      m_DriveSubsystem.arcadeDriveSquared(0.0,-rot);

    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
  }
  
