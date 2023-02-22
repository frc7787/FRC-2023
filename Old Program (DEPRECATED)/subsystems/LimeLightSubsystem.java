package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLightSubsystem extends SubsystemBase{
    static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry Tx = table.getEntry("tx");
    NetworkTableEntry Ty = table.getEntry("ty");
    NetworkTableEntry Ta = table.getEntry("ta");
    static NetworkTableEntry Pipeline = table.getEntry("pipeline");
    public void getTargets(){
        //Brandon Feb 15: I solved the error you were having with the smartDashboard. 
        //It needs to be called within a method, not within the construction of the Class.
        //you should probably move this into the Subsystems folder too.
        
        //read values periodically
        double x = Tx.getDouble(0.0);
        double y = Ty.getDouble(0.0);
        double area = Ta.getDouble(0.0);

        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
    }
    public static void setPipeline(int m_pipeLine){
        Pipeline.setNumber(m_pipeLine);
    } 
}