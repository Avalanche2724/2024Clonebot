package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SysIdRoutines;

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
          .withSlot0(new Slot0Configs().withKS(0.195).withKV(0.126).withKP(0.36));
  private static final TalonFXConfiguration MOTOR_CONFIG_BOTTOM = MOTOR_CONFIG_TOP;
  private final TalonFX topMotor;
  private final TalonFX bottomMotor;
  private final VelocityVoltage controlTop;
  private final VelocityVoltage controlBottom;

  public enum ShootingSpeed {
    AMP(new Speeds(350, 950)),
    SUBWOOFER(new Speeds(1400, 2800));

    public record Speeds(double top, double bottom) {}

    public Speeds speeds;

    ShootingSpeed(Speeds speeds) {}
  }

  private static final double CLOSED_LOOP_ALLOWABLE_ERROR = 50;

  public Shooter() {
    topMotor = new TalonFX(TALONFX_ID_TOP);
    bottomMotor = new TalonFX(TALONFX_ID_BOTTOM);
    topMotor.getConfigurator().apply(MOTOR_CONFIG_TOP);
    bottomMotor.getConfigurator().apply(MOTOR_CONFIG_BOTTOM);
    controlTop = new VelocityVoltage(0).withSlot(0);
    controlBottom = new VelocityVoltage(0).withSlot(0);
    sysIdRoutines = new SysIdRoutines.SingleMotor(this, topMotor);
    setDefaultCommand(stopCmd());
  }

  private void motorStop() {
    /* topMotor.setControl(controlTop.withVelocity(0));
    bottomMotor.setControl(controlBottom.withVelocity(0));*/
    topMotor.set(0);
    bottomMotor.set(0);
  }

  // private ShootingSpeed.Speeds targetSpeeds; // unused, remove later

  public boolean atDesiredSpeeds() {
    return Math.abs(topMotor.getClosedLoopError().getValueAsDouble()) < CLOSED_LOOP_ALLOWABLE_ERROR
        && Math.abs(bottomMotor.getClosedLoopError().getValueAsDouble())
            < CLOSED_LOOP_ALLOWABLE_ERROR;
  }

  private void runWithSpeed(ShootingSpeed.Speeds speed) {
    // targetSpeeds = speed;
    topMotor.setControl(controlTop.withVelocity(speed.top));
    bottomMotor.setControl(controlBottom.withVelocity(speed.bottom));
  }

  public Command speedCmd(ShootingSpeed.Speeds speed) {
    return run(() -> runWithSpeed(speed));
  }

  public Command stopCmd() {
    return run(this::motorStop);
  }

  public SysIdRoutines.SingleMotor sysIdRoutines;
}
