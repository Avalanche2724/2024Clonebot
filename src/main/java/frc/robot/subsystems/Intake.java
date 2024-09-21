package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Intake extends SubsystemBase {
  private static final int TALONFX_ID = 14;

  private static final TalonFXConfiguration MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(40) // basically chosen based on randomly guessing™
                  .withStatorCurrentLimitEnable(true));

  private final TalonFX motor = new TalonFX(TALONFX_ID);
  private final VoltageOut control = new VoltageOut(0).withEnableFOC(true);

  private final LinearFilter averageCurrent = LinearFilter.movingAverage(4);
  private double linearFilterLastValue = 0;
  public final Trigger intakeCurrentUp =
      new Trigger(() -> linearFilterLastValue > 15).debounce(0.2, DebounceType.kBoth);
  public final Trigger intakeCurrentUp2 =
      new Trigger(() -> linearFilterLastValue > 15).debounce(0.35, DebounceType.kBoth);

  public Intake() {
    motor.getConfigurator().apply(MOTOR_CONFIG);
    motor.getStatorCurrent().setUpdateFrequency(100);

    setDefaultCommand(stopCmd());
  }

  public void periodic() {
    double current = motor.getStatorCurrent().getValueAsDouble();
    SmartDashboard.putNumber(
        "Intake avg current", linearFilterLastValue = averageCurrent.calculate(current));
    SmartDashboard.putNumber("Intake velocity", motor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Intake voltage", motor.getMotorVoltage().getValueAsDouble());
  }

  private void setMotor(double volts) {
    motor.setControl(control.withOutput(volts));
  }

  private void setMotor(Output out) {
    setMotor(out.volts);
  }

  private Command motorSpeedCmd(Output out) {
    return startEnd(() -> setMotor(out), () -> setMotor(Output.STOP))
        .withName("IntakeSpeed" + out.name());
  }

  public Command intakeCmd() {
    return motorSpeedCmd(Output.INTAKE);
  }

  //public Command superIntakeCmd() {
  //  return motorSpeedCmd(Output.SUPERINTAKE);
  //}

  public Command ejectCmd() {
    return motorSpeedCmd(Output.EJECT);
  }

  public Command stopCmd() {
    return motorSpeedCmd(Output.STOP);
  }

  public enum Output { // in units of volts
    INTAKE(3),
    //SUPERINTAKE(9),
    EJECT(-6),
    STOP(0);

    public final double volts;

    Output(double volts) {
      this.volts = volts;
    }
  }
}
