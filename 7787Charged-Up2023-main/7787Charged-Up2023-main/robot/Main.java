package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
  private Main() {}

  // Don't touch unless you are 100% sure you know what your are doing.
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}