package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class NavXSubsystem extends SubsystemBase{
    AHRS ahrs;

    public void ahrsInit() {
        try {
			/***********************************************************************
			 * navX-MXP:
			 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.            
			 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * navX-Micro:
			 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
			 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * Multiple navX-model devices on a single robot are supported.
			 ************************************************************************/
            ahrs = new AHRS(SPI.Port.kMXP); 
        } catch (RuntimeException ex ) {
            SmartDashboard.putString("Error instantiating navX MXP:  ", ex.getMessage());
        }
    }

    public double navXPitch(){
        double pitch = ahrs.getPitch();
        return pitch;
    }
    public double navXYaw(){
        double yaw = ahrs.getYaw();
        return yaw;
    }
    public double navXRoll(){
        double roll = ahrs.getRoll();
        return roll;
    }

}
