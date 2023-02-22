package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    public static float[] getArmPosition(float Theta1, float Theta2) {

        // Calculations done in Radians
        float Hypotenus = 2 * ArmConstants.ARM_LENGTH1_INCHES * (float) Math.sin(Theta2 / 2 * Math.PI / 180);
        float Theta1_1 = (float) Math.asin((ArmConstants.ARM_LENGTH1_INCHES / Hypotenus) * Math.sin(Theta2 * Math.PI / 180));
        float Theta1_2 = Theta1 - Theta1_1;
    
        // Define an arry of values
        float[] HeightAndRadius = new float[1];
          HeightAndRadius[0] = Hypotenus * (float) Math.cos(Theta1_2 * Math.PI / 180); 
          HeightAndRadius[1] = Hypotenus * (float) Math.sin(Theta1_2 * Math.PI / 180);
          
        // Return values
          return HeightAndRadius;
      }
    
      public static float[] GetArmAngle(float ARMLENGTH1, float ARMLENGTH2, float Hypotenus, float Theta1, float Theta2) {
        if (Theta1 > 30 || Theta1 < 135 || Theta2 > 30 || Theta2 < 180) { 
          // Calculations done in Radians
          float[] Thetas = new float[2];
            Thetas[0] = (float) Math.acos((Math.pow(ARMLENGTH1, 2) + Math.pow(Hypotenus, 2) - Math.pow(ARMLENGTH2, 2)) / (2 * ARMLENGTH1 * Hypotenus));
            Thetas[1] = (float) Math.acos((Math.pow(ARMLENGTH1, 2) + Math.pow(ARMLENGTH2, 2) - Math.pow(Hypotenus, 2)) / (2 * ARMLENGTH1 * ARMLENGTH2)); 
            return Thetas;
            // Return the values
            
          }
        else {
          System.out.println("Please enter a valid angle.");
          return null;
          }
      }
    
      public static void main(String[] args) {
        // Test for arm positions
        float[] armPosition = getArmPosition(90, 90);
        System.out.println("Arm Position: (" + armPosition[0] + ", " + armPosition[1] + ")");
        
        // Test for arm angles
        float[] armAngles = GetArmAngle(40, 40, armPosition[0], 90, 90);
        System.out.println("Arm Angles: (" + armAngles[0] + ", " + armAngles[1] + ", " + armAngles[2] + ")");
      }

      public CommandBase armToPositionPolar(double Radius, double Height, double Azimuth) {
        return run(()->getArmPosition(0, 0));
      } 



}

