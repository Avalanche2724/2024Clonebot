package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShootingSpeed;

public class AutoCommands {
  public static RobotContainer bot;

  public static Command intake() {
    return bot.intakeUntilNote();
  }

  public static Command shoot() {
    return bot.shootyShoot(() -> ShootingSpeed.SUBWOOFER.speeds); // make better later
  }
}
