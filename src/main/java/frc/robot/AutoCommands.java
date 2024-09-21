package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Shooter.ShootingSpeed;
import java.util.Map;

public class AutoCommands {
  public static RobotContainer bot;

  public static Command intake() {
    return bot.intakeUntilNote();
  }

  public static Command shoot() {
    return bot.shootyShoot(() -> ShootingSpeed.SUBWOOFER.speeds); // make better later
  }

  public static Command superShoot() {
    return bot.preventStuckNote()
        .raceWith(Commands.waitSeconds(2))
        .andThen(bot.shootyShoot(() -> ShootingSpeed.SUBWOOFER.speeds)); // make better later
  }

  public static void register() {
    NamedCommands.registerCommands(
        Map.of(
            "shoot", AutoCommands.shoot(),
            "intake", AutoCommands.intake(),
            "superShoot", AutoCommands.superShoot()));
  }
}
