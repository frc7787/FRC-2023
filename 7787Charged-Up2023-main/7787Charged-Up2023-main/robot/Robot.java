// Cleaned
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.NavXSubsystem;

public class Robot extends TimedRobot {
  private NavXSubsystem m_navXSubsystem;
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

    @Override
    public void robotInit() {
        // Initizile robot container, and NavX
        m_robotContainer = new RobotContainer();
        m_navXSubsystem = m_robotContainer.getNavXSubsystem();
        m_navXSubsystem.ahrsInit();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }


    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
            System.out.println("Scheduled!!");
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        m_robotContainer.configureDefaultCommands(); 

        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all commands
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

}
