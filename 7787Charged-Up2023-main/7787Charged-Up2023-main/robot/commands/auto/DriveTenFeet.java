package frc.robot.commands.auto;

//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTenFeet extends SequentialCommandGroup {
    
  
    private DriveSubsystem m_DriveSubsystem;

public DriveTenFeet(DriveSubsystem driveSubsystem) {
        
 
    
    this.m_DriveSubsystem = driveSubsystem;

    addCommands(
        new SequentialCommandGroup(
            new RunMotorsToPosition(m_DriveSubsystem, 20),
            new RunMotorTimed(m_DriveSubsystem, 3,0),
            new RunMotorsToPosition(m_DriveSubsystem, 40,0.9),
            new RunMotorTimed(m_DriveSubsystem, 3,0),
            new RunMotorsToPosition(m_DriveSubsystem, 60)
        )
    );

}
}


// public class DriveAndParkBrandon extends SequentialCommandGroup{
//     public DriveAndParkBrandon(DriveSubsystem m_DriveSubsystem) {
//         addCommands(
//             new SequentialCommandGroup(
//                 new RunMotor(m_DriveSubsystem,5)//,
//             //new StopMotor(m_DriveSubsystem)
//             );
//         )
        
        
            
            
            
//     }

    
    
// }
