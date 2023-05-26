package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

// Class the Stop Motor
public class StopMotor extends CommandBase {

    // Suppress the warnings that we are too lazy to fix.
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    // Create instance of the drive subsystem
    private DriveSubsystem m_subsystem;
    
    // Adds the dependancies of the command
    public StopMotor(DriveSubsystem subsystem) {
      m_subsystem = subsystem;
      addRequirements(subsystem);
      m_subsystem.arcadeDriveSquared((double)0,(double)0);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      // Command to stop the motor
      m_subsystem.arcadeDriveSquared((double)0,(double)0);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return true;
    }
  }
  
