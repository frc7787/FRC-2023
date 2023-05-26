package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MotorAutoTest;

public class MotorCmd extends CommandBase{
    private MotorAutoTest subsystem;
    private boolean isComplete = false;
    private double startTime;
    private double pwr;

    // Creates the dependencies for the command.
    public MotorCmd(MotorAutoTest m_subsystem, double m_pwr) {

      subsystem = m_subsystem;
      addRequirements(subsystem);
      pwr = m_pwr;
    }
  
    // Called when the command is initialized
    @Override
    public void initialize() {
        startTime = Timer.getMatchTime();
    }
    
    // Called every time the scheduler runs
    @Override
    public void execute() {
        if (Timer.getMatchTime()> 3+startTime) {
            isComplete = true;
        }
        else {
            subsystem.pwrSet(pwr);
            Timer.delay(3);
            isComplete = true;
        }

      // Command to run the motor
      SmartDashboard.putBoolean("isComplete", isComplete);
      SmartDashboard.putNumber(getName(), startTime);
      SmartDashboard.putNumber(getName(), Timer.getMatchTime());

      
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return isComplete;
    }
    
}
