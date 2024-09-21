package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Indexer extends SubsystemBase {
  private static final int TALONFX_ID = 5;
  private static final int LEFT_SENSOR = 0;
  private static final int RIGHT_SENSOR = 1;

  private static final TalonFXConfiguration MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.CounterClockwise_Positive))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(40) // basically chosen based on guessing™
                  .withStatorCurrentLimitEnable(true));

  public final Sensors sensors = new Sensors();
  private final TalonFX motor = new TalonFX(TALONFX_ID);
  private final VoltageOut control = new VoltageOut(0).withEnableFOC(true);

  public Indexer() {
    motor.getConfigurator().apply(MOTOR_CONFIG);
    setDefaultCommand(stopCmd());
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

  public Command feedCmd() {
    return motorSpeedCmd(Output.FEED);
  }

  public Command softFeedCmd() {
    return motorSpeedCmd(Output.SOFTFEED);
  }

  public Command ejectCmd() {
    return motorSpeedCmd(Output.EJECT);
  }

  public Command stopCmd() {
    return motorSpeedCmd(Output.STOP);
  }

  public void periodic() {
    SmartDashboard.putNumber("Indexer velocity", motor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Indexer voltage", motor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Indexer voltage", motor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putBoolean("Left trigger", sensors.leftTrigger.getAsBoolean());
    SmartDashboard.putBoolean("Right trigger", sensors.rightTrigger.getAsBoolean());
  }

  // TODO: investigate using velocity, supply limits, current detection, simulation
  public enum Output { // in units of volts
    SOFTFEED(8),
    FEED(9),
    EJECT(-6),
    STOP(0);

    public final double volts;

    Output(double volts) {
      this.volts = volts;
    }
  }

  public static class Sensors {
    public DigitalInput leftSensor = new DigitalInput(LEFT_SENSOR);
    public DigitalInput rightSensor = new DigitalInput(RIGHT_SENSOR);

    /** True if left index sensor detects something */
    public Trigger leftTrigger = new Trigger(leftSensor::get).negate();

    /** True if right index sensor detects something */
    public Trigger rightTrigger = new Trigger(rightSensor::get).negate();

    /** True if a note is detected in the indexer */
    public Trigger noteDetected = leftTrigger.and(rightTrigger);
  }
}
