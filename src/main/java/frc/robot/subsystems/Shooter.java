package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.SysIdUtil;
import frc.robot.subsystems.Shooter.ShootingSpeed.Speeds;
import java.util.function.Supplier;

public class Shooter extends SubsystemBase {
  // possible TODO: telemetry, supply limits, current detection, simulation
  private static final int TALONFX_ID_TOP = 20;
  private static final int TALONFX_ID_BOTTOM = 0;

  private static final TalonFXConfiguration MOTOR_CONFIG_TOP =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withSlot0(new Slot0Configs().withKS(0.195).withKV(0.126).withKP(0.377).withKA(0.0127));
  private static final TalonFXConfiguration MOTOR_CONFIG_BOTTOM = MOTOR_CONFIG_TOP;
  private static final double CLOSED_LOOP_ALLOWABLE_ERROR = .6; // rotations per second

  private final TalonFX topMotor;
  private final TalonFX bottomMotor;
  private final VelocityVoltage controlTop;
  private final VelocityVoltage controlBottom;
  public SysIdRoutine sysIdRoutine;

  public Shooter() {
    topMotor = new TalonFX(TALONFX_ID_TOP);
    bottomMotor = new TalonFX(TALONFX_ID_BOTTOM);
    topMotor.getConfigurator().apply(MOTOR_CONFIG_TOP);
    bottomMotor.getConfigurator().apply(MOTOR_CONFIG_BOTTOM);
    controlTop = new VelocityVoltage(0).withSlot(0);
    controlBottom = new VelocityVoltage(0).withSlot(0);
    sysIdRoutine = SysIdUtil.singleMotor(this, topMotor);
    setDefaultCommand(stopCmd());
  }

  private void motorStop() {
    topMotor.set(0);
    bottomMotor.set(0);
  }

  public boolean atDesiredSpeeds() {
    return Math.abs(topMotor.getClosedLoopError().getValueAsDouble()) < CLOSED_LOOP_ALLOWABLE_ERROR
        && Math.abs(bottomMotor.getClosedLoopError().getValueAsDouble())
            < CLOSED_LOOP_ALLOWABLE_ERROR;
  }

  private void runWithSpeed(Speeds speed) {
    topMotor.setControl(controlTop.withVelocity(speed.top / 60)); // we need 2 refactor later
    bottomMotor.setControl(controlBottom.withVelocity(speed.bottom / 60));
  }

  public Command speedCmd(Speeds speed) {
    return speedCmd(() -> speed);
  }

  public Command speedCmd(
      Supplier<Speeds> speed) { // We should probably change this but that can be done later
    return run(() -> runWithSpeed(speed.get()));
  }

  public Command stopCmd() {
    return run(this::motorStop);
  }

  public void periodic() {
    SmartDashboard.putNumber(
        "Shooter closed loop error top", topMotor.getClosedLoopError().getValueAsDouble());
    SmartDashboard.putNumber(
        "Shooter closed loop error bottom", bottomMotor.getClosedLoopError().getValueAsDouble());
    SmartDashboard.putNumber("Shooter velocity top", topMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber(
        "Shooter velocity bottom", bottomMotor.getVelocity().getValueAsDouble());
  }

  public enum ShootingSpeed {
    AMP(new Speeds(400, 1000)),
    SUBWOOFER(new Speeds(1900, 3800)),
    LINESHOT(new Speeds(4000, 1800)),
    FARTHERSHOT(new Speeds(5600, 1450));

    public Speeds speeds;

    ShootingSpeed(Speeds speeds) {
      this.speeds = speeds;
    }

    public record Speeds(double top, double bottom) {}
  }
}
