package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
  // HUGE TODO: see "Preventing wheel slip" in phoenix 6 docs

  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean hasAppliedOperatorPerspective = false;

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  @Override
  public void periodic() {
    /* Periodically try to apply the operator perspective */
    /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
    /* This allows us to correct the perspective in case the robot code restarts mid-match */
    /* Otherwise, only check and apply the operator perspective if the DS is disabled */
    /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
    if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              (allianceColor) -> {
                this.setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? RedAlliancePerspectiveRotation
                        : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
              });
    }
  }

  /**
   * So basically, the other gyro reset method always assumed the robot is facing towards red
   * alliance wall This method assumes the robot is facing away from the operator regardless of
   * alliance TODO: figure out how this interacts with mid-match rio restarts?
   */
  public void resetGyroToForwardFromOperatorPointOfView() {
    var currentLocation = getState().Pose;
    var newLocation =
        new Pose2d(
            currentLocation.getTranslation(),
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? Rotation2d.fromDegrees(0)
                : Rotation2d.fromDegrees(180));
    seedFieldRelative(newLocation);
  }

  public class SysIdStuff {
    private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization =
        new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveRotation RotationCharacterization =
        new SwerveRequest.SysIdSwerveRotation();
    private final SwerveRequest.SysIdSwerveSteerGains SteerCharacterization =
        new SwerveRequest.SysIdSwerveSteerGains();

    private SysIdRoutine generateRoutine(double volts, Consumer<Measure<Voltage>> drive) {
      return new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(volts),
              null,
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(drive, null, CommandSwerveDrivetrain.this));
    }

    /* Use one of these sysidroutines for your particular test */
    private SysIdRoutine SysIdRoutineTranslation =
        generateRoutine(4, (volts) -> setControl(TranslationCharacterization.withVolts(volts)));

    private final SysIdRoutine SysIdRoutineRotation =
        generateRoutine(4, (volts) -> setControl(RotationCharacterization.withVolts(volts)));

    private final SysIdRoutine SysIdRoutineSteer =
        generateRoutine(7, (volts) -> setControl(SteerCharacterization.withVolts(volts)));

    /* Change this to the sysid routine you want to test */
    private final SysIdRoutine routineToApply = SysIdRoutineTranslation;
  }

  public SysIdStuff sysId = new SysIdStuff();
}
