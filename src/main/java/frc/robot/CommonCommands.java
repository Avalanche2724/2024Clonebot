package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Shooter.ShootingSpeed.Speeds;
import java.util.function.Supplier;

public class CommonCommands {

  private final RobotContainer bot;

  public CommonCommands(RobotContainer bot) {
    this.bot = bot;
  }

  // Intake and stop the command once note detected
  public Command intakeUntilNote() {
    return bot.intake
        .intakeCmd()
        .alongWith(bot.indexer.softFeedCmd())
        .until(bot.indexer.sensors.noteDetected);
  }

  // Command used in driver-controlled, rumbles when note detected
  // TODO: we should refactor this so that noteDetected rumbles whenever a note is in,
  // like intake.intakeCurrentUp does; initially I had done it this way because I thought
  // it would be annoying for the controller to constantly rumble, but this causes a few issues:
  // 1. doesn't work with preventStuckNote which is kinda annoying
  // 2. based on recent matches, it might actually be better to lightly rumble the controller
  // even if not intaking so that the driver is assured that there's a note
  public Command intakeUntilNoteWhileRumble() {
    return intakeUntilNote()
        .alongWith(
            Commands.run(
                () -> {
                  if (bot.indexer.sensors.noteDetected.getAsBoolean()) {
                    bot.joystick.getHID().setRumble(RumbleType.kLeftRumble, .4);
                  }
                }))
        .andThen(Commands.waitSeconds(0.2))
        .finallyDo(() -> bot.joystick.getHID().setRumble(RumbleType.kLeftRumble, 0));
  }

  // spins the intake and the indexer; used for shooting
  // we really only need to spin the indexer but this *might* help some note-stuck situations
  public Command fullIndexerAndIntakeFeed() {
    return bot.indexer.feedCmd().alongWith(bot.intake.intakeCmd());
  }

  public Command shootyShoot(Supplier<Speeds> speedy, double waitTime) {
    return bot.shooter
        .speedCmd(speedy)
        .raceWith(
            // we need to wait a bit otherwise atdesiredspeeds will return true
            // TODO: we really should fix this by checking the most recently set speeds instead
            Commands.waitSeconds(waitTime)
                .andThen(
                    Commands.waitUntil(bot.shooter::atDesiredSpeeds)
                        .andThen(fullIndexerAndIntakeFeed().raceWith(Commands.waitSeconds(1)))))
        .raceWith(Commands.waitSeconds(5))
        // see the comments on shooter::speedCmd, fix this later
        .andThen(bot.shooter.stopCmd().raceWith(Commands.waitSeconds(0.05)));
  }

  public Command shootyShoot(Supplier<Speeds> speedy) {
    return shootyShoot(speedy, 0.12); // a value smaller than 0.12 should also work
  }

  public Command autoShootyShoot(Supplier<Speeds> speedy) {
    // 2024-09-21: so, we were having some issues in auto with the bot feeding the note before
    // shooting, so we made this to give the shooter some time
    // retrospectively, this was probably due to the 4 Hz closed loop error update rate
    // however, this is now being changed to 50 Hz in Shooter (though really it would be easier to
    // just use motor velocity... soon(tm))
    // at any rate, this really shouldn't be necessary anymore
    return shootyShoot(speedy, 1);
  }

  // eject
  public Command bothEject() {
    return bot.indexer.ejectCmd().alongWith(bot.intake.ejectCmd());
  }

  // This command will attempt to intake a note, and then
  // alternate between eject and intake in hopes of getting the note not to be stuck.
  // It should terminate whenever the indexer detects a note
  public Command preventStuckNote() {
    return (bot.intake
        .intakeCmd()
        .alongWith(bot.indexer.softFeedCmd())
        .until(bot.intake.intakeCurrentUp2.or(bot.indexer.sensors.noteDetected))
        .andThen(
            bothEject()
                .raceWith(Commands.waitSeconds(0.09))
                .andThen(intakeUntilNote().raceWith(Commands.waitSeconds(0.55))))
        .repeatedly()
        .until(bot.indexer.sensors.noteDetected)
        .andThen(
            // TODO: do we really need this? the indexer and intake commands both stop on end
            bot.intake
                .stopCmd()
                .alongWith(bot.indexer.stopCmd())
                .raceWith(Commands.waitSeconds(0.03))))
        .onlyIf(bot.indexer.sensors.noteDetected.negate());
  }

  // TODO investigate whether this would be helpful
  // ... I am reading this later and have absolutely no idea what I meant? figure out later
  /*public Command intakeUntilNoteHoldIntake() {
    return intake.intakeCmd().alongWith(indexer.softFeedCmd().until(indexer.sensors.noteDetected).andThen(indexer.stopCmd()));
  }*/

}
