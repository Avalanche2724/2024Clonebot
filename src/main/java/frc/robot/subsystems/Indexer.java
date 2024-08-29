package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Indexer extends SubsystemBase {
  // TODO also: Make the sensors able to be disabled individually
  // TODO: investigate using velocity, supply limits, current detection, simulation

  // for intaking
  private static final double SOFTFEEDING_SPEED = 0.25;
  private static final double FEEDING_SPEED = 0.5;
  private static final double EJECTING_SPEED = -0.5;
  private static final int TALONFX_ID = 5;
  private static final int LEFT_SENSOR = 0;
  private static final int RIGHT_SENSOR = 1;
  private static final TalonFXConfiguration MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.CounterClockwise_Positive));
  private final TalonFX motor;
  private final DutyCycleOut control;
  private final DigitalInput leftIndexSensor;
  private final DigitalInput rightIndexSensor;
  public final Trigger bothSensorsTriggered;

  public Indexer() {
    motor = new TalonFX(TALONFX_ID);
    control = new DutyCycleOut(0);

    leftIndexSensor = new DigitalInput(LEFT_SENSOR);
    rightIndexSensor = new DigitalInput(RIGHT_SENSOR);
    bothSensorsTriggered = new Trigger(leftIndexSensor::get).and(rightIndexSensor::get);

    motor.getConfigurator().apply(MOTOR_CONFIG);

    setDefaultCommand(stopCmd());
  }

  private void motorStop() {
    motor.setControl(control.withOutput(0));
  }

  private void motorSoftFeed() {
    motor.setControl(control.withOutput(SOFTFEEDING_SPEED));
  }

  private void motorFeed() {
    motor.setControl(control.withOutput(FEEDING_SPEED));
  }

  private void motorEject() {
    motor.setControl(control.withOutput(EJECTING_SPEED));
  }

  public Command feedCmd() {
    return run(this::motorFeed);
  }

  public Command ejectCmd() {
    return run(this::motorEject);
  }

  public Command stopCmd() {
    return run(this::motorStop);
  }
}
