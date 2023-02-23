package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class StopMotor extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private DriveSubsystem m_subsystem;

    public StopMotor(DriveSubsystem subsystem) {
      m_subsystem = subsystem;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(subsystem);
      m_subsystem.arcadeDriveSquared((double)0,(double)0);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
    // Called every time the scheduler runs while the command is scheduled.

    @Override
    public void execute() {
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