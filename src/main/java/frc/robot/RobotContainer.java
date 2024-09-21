// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import java.nio.file.Files;
import java.util.Map;
import java.util.function.DoubleUnaryOperator;
import java.util.function.Supplier;

public class RobotContainer {
  // Subsystems
  public final Shooter shooter = new Shooter();
  public final Indexer indexer = new Indexer();
  public final Intake intake = new Intake();
  // It seems kinda weird that CTRE instantiates this statically
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  // Other stuff
  private static final boolean doSysID = false;
  private final SysIdRoutine routine = null; // drivetrain.sysId.routineToApply;
  public final Photon photon = new Photon();
  // Bindings
  private final CommandXboxController joystick = new CommandXboxController(0);
  public Shooter.ShootingSpeed plannedShootSpeed = Shooter.ShootingSpeed.AMP;
  private final DoubleUnaryOperator stickDeadband = (val) -> MathUtil.applyDeadband(val, 0.10);
  // Other stuff
  private final Telemetry logger = new Telemetry(drivetrain.MaxSpeed);

  public RobotContainer() {
    configureDriveBindings();
    if (doSysID) {
      configureSysIDBindings(routine);
    } else {
      configureNonDriveBindings();
    }

    AutoCommands.bot = this;
    NamedCommands.registerCommands(
        Map.of(
            "shoot", AutoCommands.shoot(),
            "intake", AutoCommands.intake()));
    drivetrain.configurePathPlanner();

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    populateAutoChooser();
    SmartDashboard.putData("CHOOSE AUTO!!!", chooser);
    // SmartDashboard.put
  }

  private void configureDriveBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () -> {
              double joystickY = -joystick.getLeftY();
              double joystickX = joystick.getLeftX();
              double rot = -joystick.getRightX();
              double leftJoystickAngle = Math.atan2(joystickY, joystickX);
              double leftJoystickDist = Math.hypot(joystickX, joystickY);

              rot = stickDeadband.applyAsDouble(rot);
              leftJoystickDist = stickDeadband.applyAsDouble(leftJoystickDist);

              double forward = Math.sin(leftJoystickAngle) * leftJoystickDist;
              double left = -(Math.cos(leftJoystickAngle) * leftJoystickDist);

              // right trigger: P control towards

              return drivetrain
                  .drive
                  .withVelocityX( // Drive forward with negative Y (forward)
                      forward * CommandSwerveDrivetrain.MaxSpeed)
                  .withVelocityY( // Drive left with negative X (left)
                      left * CommandSwerveDrivetrain.MaxSpeed)
                  .withRotationalRate( // Drive counterclockwise with negative X (left)
                      rot * CommandSwerveDrivetrain.MaxAngularRate);
            }));
    // Back button: Recenter gyro
    joystick
        .back()
        .onTrue(drivetrain.runOnce(drivetrain::resetGyroToForwardFromOperatorPointOfView));
  }

  public Command intakeUntilNote() {
    return intake.intakeCmd().alongWith(indexer.softFeedCmd()).until(indexer.sensors.noteDetected);
  }

  /*public Command superIntakeUntilNote() {
    return intake.intakeCmd().alongWith(indexer.softFeedCmd()).until(indexer.sensors.noteDetected);
  }*/

  // TODO investigate whether this would be helpful
  /*public Command intakeUntilNoteHoldIntake() {
    return intake.intakeCmd().alongWith(indexer.softFeedCmd().until(indexer.sensors.noteDetected).andThen(indexer.stopCmd()));
  }*/

  public Command intakeUntilNoteWhileRumble() {
    return intakeUntilNote()
        .alongWith(
            Commands.run(
                () -> {
                  if (indexer.sensors.noteDetected.getAsBoolean()) { // make this better later
                    joystick.getHID().setRumble(RumbleType.kLeftRumble, .4);
                  }
                }))
        .andThen(Commands.waitSeconds(0.2))
        .finallyDo(() -> joystick.getHID().setRumble(RumbleType.kLeftRumble, 0));
  }

  public Command fullIndexerAndIntakeFeed() {
    return indexer.feedCmd().alongWith(intake.intakeCmd());
  }

  public Command shootyShoot(Supplier<Speeds> speedy) {
    return shooter
        .speedCmd(speedy)
        .raceWith(
            // we need to wait a bit otherwise atdesiredspeeds will return true
            // we really could fix this by checking the most recently set speeds and we
            // should do this
            Commands.waitSeconds(0.1)
                .andThen(
                    Commands.waitUntil(
                            shooter::atDesiredSpeeds
                            //  () -> true // used for testing autos during simulation
                        )
                        .andThen(fullIndexerAndIntakeFeed().raceWith(Commands.waitSeconds(1)))))
        .raceWith(Commands.waitSeconds(5))
        .andThen(shooter.stopCmd().raceWith(Commands.waitSeconds(0.05)));
  }

  public Command bothEject() {
    return indexer.ejectCmd().alongWith(intake.ejectCmd());
  }

  public Command preventStuckNote() {
    // alternate between eject and intake in hopes of resolving issues
    return intake
        .intakeCmd()
        .alongWith(indexer.softFeedCmd())
        .until(intake.intakeCurrentUp2.or(indexer.sensors.noteDetected))
        .andThen(
            bothEject()
                .raceWith(Commands.waitSeconds(0.08))
                .andThen(intakeUntilNote().raceWith(Commands.waitSeconds(0.6))))
        .repeatedly()
        .until(indexer.sensors.noteDetected);
  }

  public Command setShootSpeedCmd(ShootingSpeed sp) {
    return Commands.runOnce(() -> plannedShootSpeed = sp);
  }

  private void configureNonDriveBindings() {

    intake.intakeCurrentUp.whileTrue(
        Commands.startEnd(
            () -> joystick.getHID().setRumble(RumbleType.kRightRumble, .4),
            () -> joystick.getHID().setRumble(RumbleType.kRightRumble, 0)));
    // Driver bindings:
    // Start button: Eject
    joystick.start().whileTrue(bothEject());

    // joystick.leftTrigger().whileTrue
    // Left bumper: Intake
    joystick.leftBumper().whileTrue(intakeUntilNoteWhileRumble());
    // Right bumper: Shoot
    joystick.rightBumper().whileTrue(shootyShoot(() -> plannedShootSpeed.speeds));
    // Left trigger: Unstuck note
    joystick.leftTrigger().whileTrue(preventStuckNote());
    // Right trigger: Spin up shoot speed without shooting
    joystick.rightTrigger().whileTrue(shooter.speedCmd(() -> plannedShootSpeed.speeds));
    // A AMP
    joystick.a().onTrue(setShootSpeedCmd(ShootingSpeed.AMP));
    // B: Shooter
    joystick.b().onTrue(setShootSpeedCmd(ShootingSpeed.SUBWOOFER));

    // Y: Shooter
    joystick.y().onTrue(setShootSpeedCmd(ShootingSpeed.FARTHERSHOT));
    // X: Shooter
    joystick.x().onTrue(setShootSpeedCmd(ShootingSpeed.LINESHOT));
  }

  public Command getAutonomousCommand() {
    // return drivetrain.getAutoPath("autopath");
    // return Commands.print("No autonomous command configured");
    /*return shootyShoot(() -> ShootingSpeed.SUBWOOFER.speeds)
    .raceWith(Commands.waitSeconds(3))
    .andThen(Commands.runOnce(drivetrain::resetGyroToForwardFromOperatorPointOfView))
    .andThen(
        drivetrain
            .applyRequest(
                () ->
                    drivetrain.drive.withVelocityX(1.5).withVelocityY(0).withRotationalRate(0))
            .raceWith(Commands.waitSeconds(4)));*/
    // return drivetrain.getAutoPath("left score auto");
    return chooser.getSelected();
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

  public void addPhotonPos() {
    var pose = photon.getEstimatedGlobalPose();

    if (pose.isPresent()) {

      var p = pose.get();
      var pose3d = p.estimatedPose;
      var pose2d = pose3d.toPose2d();

      SmartDashboard.putString("vision pose: ", pose3d.toString());
      SmartDashboard.putString("vision pose2d: ", pose2d.toString());

      if (Math.abs(pose3d.getZ()) > 0.1) {
        return;
      }
      if (pose3d.getY() < 0 || pose3d.getX() < 0) {
        return;
      }
      // todo get more accurate dimensions for field
      if (pose2d.getY() > 15 || pose2d.getX() > 15) {
        return;
      }

      double maxArea = 0;
      for (var target : p.targetsUsed) {
        maxArea = Math.max(target.getArea(), maxArea);
      }
      if (maxArea < 200) { // pixels
        return;
      }

      // System.out.println("Pose passed " + maxArea);
      drivetrain.addVisionMeasurement(pose2d, p.timestampSeconds);
    }
  }

  SendableChooser<Command> chooser = new SendableChooser<>();

  private void populateAutoChooser() {
    var deployDir = Filesystem.getDeployDirectory();
    chooser.addOption("nothing", Commands.none());
    chooser.addOption("shoot", AutoCommands.shoot());
    if (Robot.isSimulation()) {
      chooser.setDefaultOption("amp auto", drivetrain.getAutoPath("amp side auto"));
    }

    try {
      System.out.println("PATH " + deployDir.toPath().resolve("pathplanner/autos").toString());
      // Automatically list all the paths and add them all!
      Files.list(deployDir.toPath().resolve("pathplanner/autos"))
          .sorted()
          .filter(file -> !file.toString().contains("unused"))
          .forEach(
              file -> {
                try {
                  var name = file.getName(file.getNameCount() - 1).toString().replace(".auto", "");
                  chooser.addOption(name, drivetrain.getAutoPath(name));
                } catch (Exception e) {
                  SmartDashboard.putString("ERROR LOADING " + file, e.getMessage());
                }
              });
    } catch (Exception e) {
      // Add manually, even though this should literally never happen
      // Maybe it will happen, though?
      SmartDashboard.putString("WARNING", "UNABLE TO AUTOMATICALLY DO AUTOS");
      e.printStackTrace();
    }
  }
}
