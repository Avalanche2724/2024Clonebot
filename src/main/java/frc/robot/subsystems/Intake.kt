package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WrapperCommand
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.subsystems.Indexer.Output
import kotlin.math.absoluteValue

private const val TALONFX_ID = 14

class Intake : SubsystemBase() {
    private val motor = TalonFX(TALONFX_ID).apply {
        configurator.apply(TalonFXConfiguration().apply {
            MotorOutput.apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.Clockwise_Positive
            }
            CurrentLimits.apply {
                StatorCurrentLimit = 40.0
                StatorCurrentLimitEnable = true
            }
            Slot0.apply {
                kP = 0.35
            }
        })
    }

    private val control = VoltageOut(0.0).apply {
        UpdateFreqHz = 0.0
    }
    private val brake = VelocityVoltage(0.0).apply {
        UpdateFreqHz = 0.0
    }

    private val torqueCurrent = motor.torqueCurrent
    private val acceleration = motor.acceleration
    private val velocity = motor.velocity
    private val position = motor.position
    private val voltage = motor.motorVoltage
    private val signals = listOf(torqueCurrent, acceleration, velocity, position, voltage)
        .onEach { it.setUpdateFrequency(100.0) }

    val isMotorStalling = Trigger {
        acceleration.valueAsDouble.absoluteValue < 0.5
                && torqueCurrent.valueAsDouble.absoluteValue > 38
                && velocity.valueAsDouble.absoluteValue < 0.5
    }

    val isIntakeCurrentUp = Trigger {
        torqueCurrent.valueAsDouble.absoluteValue > 10
    }.debounce(0.3)


    val isMotorMovingBack = Trigger { velocity.valueAsDouble < -2 }

    init {
        defaultCommand = stopCmd()
    }

    private var loopsWithoutControlRequest = 0
    override fun periodic() {
        signals.onEach {
            it.refresh()
            SmartDashboard.putNumber("Intake " + it.name, it.valueAsDouble)
        }
        if (loopsWithoutControlRequest == 1) {
            setMotor(0.0);
        }
        loopsWithoutControlRequest++
    }

    private fun setMotor(volts: Double) {
        loopsWithoutControlRequest = 0
        motor.setControl(
            if (volts == 0.0) brake
            else control.withOutput(volts)
        )
    }

    private fun setMotor(out: Output) = setMotor(out.volts)

    fun stopMotor() = setMotor(Output.STOP)

    private fun motorSpeedCmd(out: Output): WrapperCommand =
        runEnd({ setMotor(out) }, { setMotor(Output.STOP) })
            .withName("IntakeSpeed" + out.name)

    fun intakeCmd() = motorSpeedCmd(Output.INTAKE)
    fun ejectCmd() = motorSpeedCmd(Output.EJECT)
    fun stopCmd() = motorSpeedCmd(Output.STOP)

    enum class Output(val volts: Double) {
        // in units of volts
        INTAKE(12.0),
        EJECT(-12.0),
        STOP(0.0)
    }
}
