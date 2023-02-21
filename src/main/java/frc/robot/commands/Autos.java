// I don't know what an HD is but my doctor says I have 80 of them.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
//import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static CommandBase exampleAutoDrive(DriveSubsystem subsystem) {
    return Commands.sequence(subsystem.driveDistanceCommand(3, .5), subsystem.driveDistanceCommand(-3, 0.5));
  }

  public static CommandBase exampleAutoArm(ArmSubsystem subsystem) {
    return Commands.sequence(subsystem.armToPositionPolar(36,52,0), subsystem.armToPositionPolar(36,36,0));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}

