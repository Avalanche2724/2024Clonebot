package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Shooter.ShootingSpeed;
import java.util.Map;

public class AutoCommands {
  public static RobotContainer bot;

  public static Command intake() {
    return bot.com.intakeUntilNote().andThen(Commands.run(() -> {}));
  }

  public static Command shoot() {
    return bot.com.autoShootyShoot(() -> ShootingSpeed.SUBWOOFER.speeds); // make better later
  }

  // "super shoot" does the fix-stuck-note procedure before shooting
  // it only sometimes fixes stuck notes, though :C
  public static Command superShoot() {
    return bot.com
        .preventStuckNote()
        .raceWith(Commands.waitSeconds(3))
        .andThen(bot.com.shootyShoot(() -> ShootingSpeed.SUBWOOFER.speeds)); // make better later
  }

  public static void register() {
    NamedCommands.registerCommands(
        Map.of(
            "shoot", AutoCommands.shoot(),
            "intake", AutoCommands.intake(),
            "superShoot", AutoCommands.superShoot()));
  }
}
