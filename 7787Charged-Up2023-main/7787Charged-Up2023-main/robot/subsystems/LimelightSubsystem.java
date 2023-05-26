package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LimelightSubsystem extends SubsystemBase{

    static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    static NetworkTableEntry Tx = table.getEntry("tx");
    static NetworkTableEntry Ty = table.getEntry("ty");
    static NetworkTableEntry Ta = table.getEntry("ta");
    static NetworkTableEntry Pipeline = table.getEntry("pipeline");
    static NetworkTableEntry Tv = table.getEntry("tv"); //are there any valid targets
    
    public  double[] getTargets(){

        //read values periodically
        double x = Tx.getDouble(0.0);
        double y = Ty.getDouble(0.0);
        double area = Ta.getDouble(0.0);
        double[] returnArray= {x,y,area};

        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);

        return returnArray;
    }
    public static void setPipeline(int m_pipeLine) {
        Pipeline.setNumber(m_pipeLine);
    } 
    
    public boolean hasTargets(){
        boolean returnBool =false;
        if((double)Tv.getNumber(0)>0){//for some reason tv.getnumber must be mapped to double, not int or boolean
            returnBool=true;
        }
        return returnBool;
    }

    public CommandBase getTargetsCommand() {
        return runOnce( ()->getTargets());
    }

    public CommandBase checkForTargetsCommand() {
        return runOnce( ()->hasTargets());
    }

    public CommandBase setPipeline9Command() {
        return runOnce( ()->this.setPipeline(9));
    }
    
    public CommandBase exampleMethodCommand() {
        return runOnce(
            () -> setPipeline(7));
    }
}