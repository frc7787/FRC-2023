package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.DriveSubsystem;

public class DriveAndParkBrandon extends SequentialCommandGroup {
    
  
    private DriveSubsystem m_DriveSubsystem;

public DriveAndParkBrandon(DriveSubsystem driveSubsystem) {
        
 
    
    this.m_DriveSubsystem = driveSubsystem;

    addCommands(
        new SequentialCommandGroup(
            new RunMotorTimed(m_DriveSubsystem, 2),
            new RunMotorTimed(m_DriveSubsystem, 2)
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
