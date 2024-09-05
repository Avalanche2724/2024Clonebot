package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.Consumer;

public class SysIdUtil {
  public static SysIdRoutine generateRoutine(
      Subsystem subsystem, double volts, Consumer<Measure<Voltage>> drive) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(volts),
            null,
            (state) -> SignalLogger.writeString("state", state.toString())),
        new SysIdRoutine.Mechanism(drive, null, subsystem));
  }

  public static SysIdRoutine singleMotor(Subsystem subsystem, TalonFX motor) {
    return SysIdUtil.generateRoutine(
        subsystem, 6, (volts) -> motor.setControl(new VoltageOut(0).withOutput(volts.in(Volts))));
  }
}
