package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunMotorTimed extends CommandBase {
    
    // Ignores the warnings that we are two lazy to fix.
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    // Creates an instance of the drive subsystem
    private DriveSubsystem m_subsystem;

    private Timer timer1;
    private double timerDuration;
    private boolean timerExpired=false;
    private static double power;

    
    
    // Creates the dependencies for the command.

    public RunMotorTimed(DriveSubsystem subsystem) {
      timerDuration=1;
      timer1= new Timer();
      power = 0.8;

      m_subsystem = subsystem;
      addRequirements(m_subsystem);
      //m_subsystem.arcadeDriveSquared(1.0,0.0);
    }
    
    public RunMotorTimed(DriveSubsystem subsystem, double l_seconds) {
      timerDuration =l_seconds;
      timer1= new Timer();
      power =0.8;

      m_subsystem = subsystem;
      addRequirements(m_subsystem);
      //m_subsystem.arcadeDriveSquared(1.0,0.0);
    }
    public RunMotorTimed(DriveSubsystem subsystem, double l_seconds, double l_power) {
      timerDuration =l_seconds;
      timer1= new Timer();
      power =l_power ;

      m_subsystem = subsystem;
      addRequirements(m_subsystem);
      //m_subsystem.arcadeDriveSquared(1.0,0.0);
    }
  
    // Called when the command is initialized
    @Override
    public void initialize() {
      timer1.reset();
      timer1.start();
    }
    
    // Called every time the scheduler runs
    @Override
    public void execute() {
      if (timer1.hasElapsed(timerDuration)){
        timerExpired=true;
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
      return timerExpired;
    }
  }
  
