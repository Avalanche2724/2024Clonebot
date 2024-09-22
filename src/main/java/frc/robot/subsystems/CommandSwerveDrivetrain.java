package frc.robot.subsystems;

import static frc.robot.SysIdUtil.generateRoutine;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
  // TODO: re-implement brake and pointat requests into controls
  // Stuff for controls:
  public static final double MaxSpeed =
      TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  public static final double MaxAngularRate = 2.5 * Math.PI;
  public final FieldCentricButBetter drive =
      (FieldCentricButBetter)
          new FieldCentricButBetter()
              // NOTE: stricter deadbands implemented in controls
              .withDeadband(MaxSpeed * 0.001)
              .withRotationalDeadband(MaxAngularRate * 0.001)
              .withDriveRequestType(
                  DriveRequestType.OpenLoopVoltage); // I want field-centric driving open loop

  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean hasAppliedOperatorPerspective = false;

  private final SwerveRequest.ApplyChassisSpeeds AutoRequest =
      new SwerveRequest.ApplyChassisSpeeds();

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

  public void configurePathPlanner() {
    double driveBaseRadius = 0;
    for (var moduleLocation : m_moduleLocations) {
      driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
    }

    AutoBuilder.configureHolonomic(
        () -> this.getState().Pose, // Supplier of current robot pose
        this::seedFieldRelative, // Consumer for seeding pose against auto
        this::getCurrentRobotChassisSpeeds,
        (speeds) -> this.setControl(AutoRequest.withSpeeds(speeds)),
        // Consumer of ChassisSpeeds to drive the robot
        new HolonomicPathFollowerConfig(
            new PIDConstants(10, 0, 0), // should we tune this? idk
            new PIDConstants(10, 0, 0),
            TunerConstants.kSpeedAt12VoltsMps,
            driveBaseRadius,
            new ReplanningConfig()),
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        // Assume the path needs to be flipped for Red vs Blue, this is normally the case
        this); // Subsystem for requirements
  }

  /*public void followPathCommand(String pathName) { // ignore this
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    // path =
  }*/

  public Command getAutoPath(String pathName) {
    return new PathPlannerAuto(pathName);
  }

  public ChassisSpeeds getCurrentRobotChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }

  // this desaturates wheel speeds, unlike the ctre api
  public class FieldCentricButBetter extends FieldCentric {
    @Override
    public StatusCode apply(
        SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
      double toApplyX = VelocityX;
      double toApplyY = VelocityY;
      if (ForwardReference == SwerveRequest.ForwardReference.OperatorPerspective) {
        /* If we're operator perspective, modify the X/Y translation by the angle */
        Translation2d tmp = new Translation2d(toApplyX, toApplyY);
        tmp = tmp.rotateBy(parameters.operatorForwardDirection);
        toApplyX = tmp.getX();
        toApplyY = tmp.getY();
      }
      double toApplyOmega = RotationalRate;
      if (Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < Deadband) {
        toApplyX = 0;
        toApplyY = 0;
      }
      if (Math.abs(toApplyOmega) < RotationalDeadband) {
        toApplyOmega = 0;
      }

      ChassisSpeeds speeds =
          ChassisSpeeds.discretize(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  toApplyX, toApplyY, toApplyOmega, parameters.currentPose.getRotation()),
              parameters.updatePeriod);
      var states = parameters.kinematics.toSwerveModuleStates(speeds, CenterOfRotation);
      // this is the line that ctre forgot (hopefully it's correct):
      SwerveDriveKinematics.desaturateWheelSpeeds(
          states, speeds, MaxSpeed, MaxSpeed, MaxAngularRate);

      for (int i = 0; i < modulesToApply.length; ++i) {
        modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
      }

      return StatusCode.OK;
    }
  }

  public class SysIdStuff {
    private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization =
        new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveRotation RotationCharacterization =
        new SwerveRequest.SysIdSwerveRotation();
    private final SwerveRequest.SysIdSwerveSteerGains SteerCharacterization =
        new SwerveRequest.SysIdSwerveSteerGains();

    /* Use one of these sysidroutines for your particular test */
    private SysIdRoutine SysIdRoutineTranslation =
        generateRoutine(
            CommandSwerveDrivetrain.this,
            4,
            (volts) -> setControl(TranslationCharacterization.withVolts(volts)));

    private final SysIdRoutine SysIdRoutineRotation =
        generateRoutine(
            CommandSwerveDrivetrain.this,
            4,
            (volts) -> setControl(RotationCharacterization.withVolts(volts)));

    private final SysIdRoutine SysIdRoutineSteer =
        generateRoutine(
            CommandSwerveDrivetrain.this,
            7,
            (volts) -> setControl(SteerCharacterization.withVolts(volts)));

    /* Change this to the sysid routine you want to test */
    public final SysIdRoutine routineToApply = SysIdRoutineRotation;
  }

  public SysIdStuff sysId = new SysIdStuff();

  /*public Command pidToLocation(Pose2d target) {

  }*/
}
