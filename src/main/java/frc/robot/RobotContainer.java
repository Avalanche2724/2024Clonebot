// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShootingSpeed;
import frc.robot.subsystems.Shooter.ShootingSpeed.Speeds;
import java.util.function.DoubleUnaryOperator;
import java.util.function.Supplier;

public class RobotContainer {
  // Subsystems
  public final Shooter shooter = new Shooter();
  public final Indexer indexer = new Indexer();
  public final Intake intake = new Intake();
  // why is this instantiated statically???? whose idea was this
  // i feel like this will cause some sort of weird issue in the future
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  // Other stuff
  private static final boolean doSysID = false;
  private final SysIdRoutine routine = drivetrain.sysId.routineToApply;

  // Bindings
  private final CommandXboxController joystick = new CommandXboxController(0);
  private Shooter.ShootingSpeed plannedShootSpeed = Shooter.ShootingSpeed.AMP;
  // Other stuff
  private final Telemetry logger = new Telemetry(drivetrain.MaxSpeed);

  public RobotContainer() {
    configureDriveBindings();
    if (doSysID) {
      configureSysIDBindings(routine);
    } else {
      configureNonDriveBindings();
    }

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  private void configureDriveBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () -> {
              double joystickY = -joystick.getLeftY();
              double joystickX = joystick.getLeftX();
              double rot = -joystick.getRightX();

              DoubleUnaryOperator transformThing = (val) -> // uses a polynomial to scale input
                  // this also applies deadbands
                  Math.abs(val) < 0.1
                      ? 0
                      : Math.copySign(Math.pow(val, 2), val) * 0.5 + val * 0.5;
              // : val;
              double leftJoystickAngle = Math.atan2(joystickY, joystickX);
              double leftJoystickDist = Math.hypot(joystickX, joystickY);

              rot = transformThing.applyAsDouble(rot);
              leftJoystickDist = transformThing.applyAsDouble(leftJoystickDist);

              double forward = Math.sin(leftJoystickAngle) * leftJoystickDist;
              double left = -(Math.cos(leftJoystickAngle) * leftJoystickDist);

              return drivetrain
                  .drive
                  .withVelocityX(
                      forward
                          * CommandSwerveDrivetrain
                          .MaxSpeed) // Drive forward with negative Y (forward)
                  .withVelocityY(
                      left * CommandSwerveDrivetrain.MaxSpeed) // Drive left with negative X (left)
                  .withRotationalRate(
                      rot
                          * CommandSwerveDrivetrain
                          .MaxAngularRate); // Drive counterclockwise with negative X (left)
            }));
    // Back button: Recenter gyro
    joystick
        .back()
        .onTrue(drivetrain.runOnce(drivetrain::resetGyroToForwardFromOperatorPointOfView));
  }

  public Command intakeUntilNote() {
    return intake
        .intakeCmd()
        .alongWith(indexer.softFeedCmd())
        .until(indexer.sensors.noteDetected)
  }
  public Command shootyShoot(Supplier<Speeds> speedy) {
    return shooter
        .speedCmd(speedy)
        .alongWith(
            // we need to wait a bit otherwise atdesiredspeeds will return true
            // we really could fix this by checking the most recently set speeds and we
            // should
            // do this
            Commands.waitSeconds(0.1)
                .andThen(
                    Commands.waitUntil(shooter::atDesiredSpeeds)
                        .andThen(indexer.feedCmd())));
  }

  private void configureNonDriveBindings() {
    // Driver bindings:
    // Start button: Eject
    joystick.start().whileTrue(indexer.ejectCmd().alongWith(intake.ejectCmd()));

    // Left bumper: Intake
    joystick
        .leftBumper()
        .whileTrue(
            intakeUntilNote());
    // Right bumper: Shoot
    joystick
        .rightBumper()
        .whileTrue(shootyShoot(()->plannedShootSpeed.speeds));

    // A AMP
    joystick
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  plannedShootSpeed = Shooter.ShootingSpeed.AMP;
                }));
    // B: Shooter
    joystick
        .b()
        .onTrue(
            Commands.runOnce(
                () -> {
                  plannedShootSpeed = Shooter.ShootingSpeed.SUBWOOFER;
                }));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  private void configureSysIDBindings(SysIdRoutine routine) {
    /* Manually start logging with left bumper before running any tests,
     * and stop logging with right bumper after we're done with ALL tests.
     * This isn't necessary but is convenient to reduce the size of the hoot file */
    joystick.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
    joystick.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

    /*
     * Joystick Y = quasistatic forward
     * Joystick A = quasistatic reverse
     * Joystick B = dynamic forward
     * Joystick X = dynamic reverse
     */
    joystick.y().whileTrue(routine.quasistatic(SysIdRoutine.Direction.kForward));
    joystick.a().whileTrue(routine.quasistatic(SysIdRoutine.Direction.kReverse));
    joystick.b().whileTrue(routine.dynamic(SysIdRoutine.Direction.kForward));
    joystick.x().whileTrue(routine.dynamic(SysIdRoutine.Direction.kReverse));
  }
}
