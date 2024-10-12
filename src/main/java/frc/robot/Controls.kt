package frc.robot

import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.GenericHID.RumbleType
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.subsystems.CommandSwerveDrivetrain
import frc.robot.subsystems.Shooter.ShootingSpeed
import java.util.function.DoubleUnaryOperator
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

class Controls(bot: RobotContainer) {
    private val intake = bot.intake
    private val indexer = bot.indexer
    private val shooter = bot.shooter
    private val drivetrain = bot.drivetrain
    private val joystick = CommandXboxController(0)
    val com = bot.com

    /** Stick deadband, can also be used to apply polynomial scale to joystick  */
    private val stickDeadband =
        DoubleUnaryOperator { value: Double -> MathUtil.applyDeadband(value, 0.10) }
    // Note: applied to magnitude of strafing stick

    private var plannedShootSpeed: ShootingSpeed = ShootingSpeed.AMP

    init {
        SmartDashboard.putData("planned shot speed") {
            it.addStringProperty(
                "speed",
                { plannedShootSpeed.name },
                null
            )
        }
    }

    fun configureDriveBindings() {
        drivetrain.defaultCommand = drivetrain.applyRequest {
            val joystickY = -joystick.leftY
            val joystickX = joystick.leftX
            //0.5
            var joystickRotation = -joystick.rightX
            val leftJoystickAngle = atan2(joystickY, joystickX)
            var leftJoystickDist = hypot(joystickX, joystickY)

            joystickRotation = stickDeadband.applyAsDouble(joystickRotation)
            leftJoystickDist = stickDeadband.applyAsDouble(leftJoystickDist)

            val forward = sin(leftJoystickAngle) * leftJoystickDist
            val left = -(cos(leftJoystickAngle) * leftJoystickDist)

            drivetrain.teleopDriveRequest
                .withVelocityX( // Drive forward with negative Y (forward)
                    forward * CommandSwerveDrivetrain.MAX_SPEED
                )
                .withVelocityY( // Drive left with negative X (left)
                    left * CommandSwerveDrivetrain.MAX_SPEED
                )
                .withRotationalRate( // Drive counterclockwise with negative X (left)
                    joystickRotation * CommandSwerveDrivetrain.MAX_ANGLE_RATE
                )
        }

        // Back button: Recenter gyro
        // Note that the battery must be at the back of the robot from the driver's POV
        joystick.back()
            .onTrue(drivetrain.runOnce(drivetrain::resetGyroToForwardFromOperatorPointOfView))
    }

    private fun CommandXboxController.rumble(type: RumbleType, value: Double): Command =
        Commands.startEnd(
            { hid.setRumble(type, value) },
            { hid.setRumble(type, 0.0) })

    fun configureNonDriveBindings() {
        with(joystick) {
            // Rumble if the intake current is high
            intake.isIntakeCurrentUp.whileTrue(rumble(RumbleType.kLeftRumble, 0.3))
            // Rumble if note detected
            indexer.noteDetected.whileTrue(rumble(RumbleType.kRightRumble, 0.5))

            // Driver bindings:
            // Start button: Eject
            start().whileTrue(com.bothEject())
            // Left bumper: Intake
            leftBumper().whileTrue(com.intakeUntilNote())
            // Right bumper: Shoot
            rightBumper().whileTrue(com.simpleShoot { plannedShootSpeed.speeds })
            // Right trigger: Shoot but better!
            rightTrigger().whileTrue(com.teleopShoot())
            // Left trigger: BETTER intake
            leftTrigger().whileTrue(com.superIntake())

            fun Trigger.shootSpeed(s: ShootingSpeed) = onTrue(runOnce({ plannedShootSpeed = s }))
            // A: AMP
            a().shootSpeed(ShootingSpeed.AMP)
            // B: Shooter
            b().shootSpeed(ShootingSpeed.SUBWOOFER)

            y().shootSpeed(ShootingSpeed.AUTOSHOT)

            // Unused:
            // Y: Shooter
            y().shootSpeed(ShootingSpeed.FARTHERSHOT)
            // X: Shooter
            ///x().shootSpeed(ShootingSpeed.LINESHOT)
        }
    }


    fun configureSysIDBindings(routine: SysIdRoutine) {
        with(joystick) {
            leftBumper().onTrue(runOnce(SignalLogger::start))
            rightBumper().onTrue(runOnce(SignalLogger::stop))
            a().whileTrue(routine.quasistatic(SysIdRoutine.Direction.kForward))
            b().whileTrue(routine.quasistatic(SysIdRoutine.Direction.kReverse))
            x().whileTrue(routine.dynamic(SysIdRoutine.Direction.kForward))
            y().whileTrue(routine.dynamic(SysIdRoutine.Direction.kReverse))
        }
    }
}