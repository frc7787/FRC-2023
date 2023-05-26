package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunMotorsToPosition extends CommandBase {
    
    // Ignores the warnings that we are two lazy to fix.
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    // Creates an instance of the drive subsystem
    private DriveSubsystem m_subsystem;

    
    private double targetInches;
    private boolean distanceReached=false;
    private static double power;

    
    
    // Creates the dependencies for the command.


    
    public RunMotorsToPosition(DriveSubsystem subsystem, double l_TargetInches) {
      targetInches=l_TargetInches;

      power =0.8;

      m_subsystem = subsystem;
      addRequirements(m_subsystem);
      //m_subsystem.arcadeDriveSquared(1.0,0.0);
    }
    public RunMotorsToPosition(DriveSubsystem subsystem, double l_TargetInches, double l_power) {
      targetInches=l_TargetInches;

      power =l_power ;

      m_subsystem = subsystem;
      addRequirements(m_subsystem);
      //m_subsystem.arcadeDriveSquared(1.0,0.0);
    }
  
    // Called when the command is initialized
    @Override
    public void initialize() {
      this.m_subsystem.getLeftEncoderDistance();
    }
    
    // Called every time the scheduler runs
    @Override
    public void execute() {
      double leftEncoderDistance= this.m_subsystem.getLeftEncoderDistance();
      if (leftEncoderDistance>=targetInches){
        distanceReached=true;
      }

      // Command to run the motor
      m_subsystem.arcadeDrive(power,0.0);
      
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return distanceReached;
    }
  }
  
