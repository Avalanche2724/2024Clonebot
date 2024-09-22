// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
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
import java.nio.file.Files;
import java.util.function.DoubleUnaryOperator;

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
  private final Telemetry logger = new Telemetry(drivetrain.MaxSpeed);
  public CommonCommands com = new CommonCommands(this);
  // Bindings
  public final CommandXboxController joystick = new CommandXboxController(0);
  public Shooter.ShootingSpeed plannedShootSpeed = Shooter.ShootingSpeed.AMP;

  /** Stick deadband, can also be used to apply polynomial scale to joystick */
  private final DoubleUnaryOperator stickDeadband = (val) -> MathUtil.applyDeadband(val, 0.10);

  public RobotContainer() {
    // Configure controller
    configureDriveBindings();
    if (doSysID) {
      configureSysIDBindings(routine);
    } else {
      configureNonDriveBindings();
    }

    // Set up drivetrain and auto
    AutoCommands.bot = this;
    AutoCommands.register();
    drivetrain.configurePathPlanner();
    drivetrain.registerTelemetry(logger::telemeterize);
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }

    populateAutoChooser();
    SmartDashboard.putData("CHOOSE AUTO!!!", chooser);
  }

  private void configureDriveBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () -> {
              // We do a bunch of math here. We used to have a polynomial scale on the magnitude
              // of the joystick, but right now it is just used for applying deadbands
              double joystickY = -joystick.getLeftY();
              double joystickX = joystick.getLeftX();
              double rot = -joystick.getRightX();
              double leftJoystickAngle = Math.atan2(joystickY, joystickX);
              double leftJoystickDist = Math.hypot(joystickX, joystickY);

              rot = stickDeadband.applyAsDouble(rot);
              leftJoystickDist = stickDeadband.applyAsDouble(leftJoystickDist);

              double forward = Math.sin(leftJoystickAngle) * leftJoystickDist;
              double left = -(Math.cos(leftJoystickAngle) * leftJoystickDist);

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
    // Note that the battery must be at the back of the robot from the driver's POV
    joystick
        .back()
        .onTrue(drivetrain.runOnce(drivetrain::resetGyroToForwardFromOperatorPointOfView));
  }

  public Command setShootSpeedCmd(ShootingSpeed sp) {
    return Commands.runOnce(() -> plannedShootSpeed = sp);
  }

  private void configureNonDriveBindings() {
    // Rumble if the intake current is high
    // TODO abstract this better
    intake.intakeCurrentUp.whileTrue(
        Commands.startEnd(
            () -> joystick.getHID().setRumble(RumbleType.kRightRumble, .4),
            () -> joystick.getHID().setRumble(RumbleType.kRightRumble, 0)));
    // Driver bindings:
    // Start button: Eject
    joystick.start().whileTrue(com.bothEject());

    // Left bumper: Intake
    joystick.leftBumper().whileTrue(com.intakeUntilNoteWhileRumble());
    // Right bumper: Shoot
    joystick.rightBumper().whileTrue(com.shootyShoot(() -> plannedShootSpeed.speeds));
    // Left trigger: Unstuck note
    joystick.leftTrigger().whileTrue(com.preventStuckNote());
    // Right trigger: Spin up shoot speed without shooting
    // (I haven't used this yet because I keep forgetting about it ;c)
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
    // TODO clean this up probably
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
