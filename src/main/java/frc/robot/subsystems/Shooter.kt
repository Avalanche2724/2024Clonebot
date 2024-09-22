package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.Shooter.ShootingSpeed.Speeds
import frc.robot.sysIdSingleMotor
import java.util.function.Supplier
import kotlin.math.absoluteValue


// possible TODO: telemetry, supply limits, current detection, simulation
private const val TALONFX_ID_TOP = 20
private const val TALONFX_ID_BOTTOM = 0

class Shooter : SubsystemBase() {
    private val topMotor = TalonFX(TALONFX_ID_TOP).apply {
        configurator.apply(TalonFXConfiguration().apply {
            MotorOutput.apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.Clockwise_Positive
            }
            Slot0.apply {
                kS = 0.195
                kV = 0.126
                kP = 0.4
            }
        })
    }
    private val bottomMotor = TalonFX(TALONFX_ID_BOTTOM).apply {
        configurator.apply(TalonFXConfiguration().apply {
            MotorOutput.apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.Clockwise_Positive
            }
            Slot0.apply {
                kS = 0.195
                kV = 0.126
                kP = 0.4
            }
        })
    }
    private val controlTop = VelocityVoltage(0.0)
    private val controlBottom = VelocityVoltage(0.0)

    val sysIdRoutine = sysIdSingleMotor(this, topMotor)

    init {
        defaultCommand = stopCmd()
    }

    private fun motorStop() {
        topMotor.set(0.0)
        bottomMotor.set(0.0)
    }

    fun atDesiredSpeeds(speed: Supplier<Speeds>) =
        (topMotor.velocity.valueAsDouble - speed.get().top).absoluteValue < 0.5 &&
                (bottomMotor.velocity.valueAsDouble - speed.get().bottom).absoluteValue < 0.5

    fun runWithSpeed(speed: Speeds) {
        topMotor.setControl(controlTop.apply { Velocity = speed.top / 60 })
        bottomMotor.setControl(controlBottom.apply { Velocity = speed.bottom / 60 })
    }

    fun speedCmd(speed: Supplier<Speeds>) = startEnd({ runWithSpeed(speed.get()) }, ::motorStop)

    fun stopCmd() = run { this.motorStop() }

    override fun periodic() {
        SmartDashboard.putNumber("Shooter velocity top", topMotor.velocity.valueAsDouble)
        SmartDashboard.putNumber("Shooter velocity bottom", bottomMotor.velocity.valueAsDouble)
    }

    enum class ShootingSpeed(var speeds: Speeds) {
        AMP(Speeds(400.0, 1000.0)),
        SUBWOOFER(Speeds(1900.0, 3800.0)),
        LINESHOT(Speeds(4000.0, 1800.0)),
        FARTHERSHOT(Speeds(5600.0, 1450.0));

        class Speeds(val top: Double, val bottom: Double)
    }
}
