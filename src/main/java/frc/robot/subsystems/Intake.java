package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  // TODO: investigate using velocity, supply limits, current detection, simulation
  private static final double INTAKING_SPEED = 0.5;
  private static final double EJECTING_SPEED = -0.5;
  private static final int TALONFX_ID = 14;
  private static final TalonFXConfiguration MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Brake)
                  .withInverted(InvertedValue.Clockwise_Positive));
  private final TalonFX motor;
  private final DutyCycleOut control;

  public Intake() {
    motor = new TalonFX(TALONFX_ID);
    control = new DutyCycleOut(0);

    motor.getConfigurator().apply(MOTOR_CONFIG);

    setDefaultCommand(stopCmd());
  }

  private void motorStop() {
    motor.setControl(control.withOutput(0));
  }

  private void motorIntake() {
    motor.setControl(control.withOutput(INTAKING_SPEED));
  }

  private void motorEject() {
    motor.setControl(control.withOutput(EJECTING_SPEED));
  }

  public Command intakeCmd() {
    return run(this::motorIntake);
  }

  public Command ejectCmd() {
    return run(this::motorEject);
  }

  public Command stopCmd() {
    return run(this::motorStop);
  }
}
