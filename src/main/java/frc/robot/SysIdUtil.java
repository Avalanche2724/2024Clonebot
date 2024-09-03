package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SysIdUtil {
  public static SysIdRoutine singleMotor(Subsystem subsystem, TalonFX motor) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            null, // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic voltage to 4 to prevent brownout
            null, // Use default timeout (10 s)
            // Log state with Phoenix SignalLogger class
            (state) -> SignalLogger.writeString("state", state.toString())),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) ->
                motor.setControl(new VoltageOut(0).withOutput(volts.in(Volts))),
            null,
            subsystem));
  }
}
