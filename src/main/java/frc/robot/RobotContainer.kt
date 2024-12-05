// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.generated.TunerConstants.createDrivetrain
import frc.robot.subsystems.CommandSwerveDrivetrain
import frc.robot.subsystems.Indexer
import frc.robot.subsystems.Intake
import frc.robot.subsystems.Shooter
import frc.robot.subsystems.Shooter.ShootingSpeed
import org.photonvision.EstimatedRobotPose

class RobotContainer(var bot: Robot) {
    // Subsystems
    val shooter = Shooter()
    val indexer = Indexer()
    val intake = Intake()
    val drivetrain = createDrivetrain()

    // Other things
    private val vision = Vision()
    private val routine: SysIdRoutine? = shooter.sysIdRoutine//null // drivetrain.sysId.routineToApply;
    private val doSysId = false
    val logger = Telemetry(CommandSwerveDrivetrain.MAX_SPEED)

    // Other necessary things
    val com = CommonCommands(this)
    private val controls = Controls(this)
    // Make outreach mode easier to access from other subsystems
    companion object {
        private var instance: RobotContainer? = null
        //val outreachMode
        //    get() = instance?.controls?.outreachMode == true
        val voltageMultiplier = 1.0
            //get() = if (outreachMode) 0.9 else 1.0


    }
    init {
        instance = this
    }

    private val chooser: SendableChooser<Command>
    val autonomousCommand: Command
        get() = chooser.selected ?: Commands.none()

    init {
        // Configure controller
        controls.configureDriveBindings()
        if (doSysId && routine != null) {
            controls.configureSysIDBindings(routine)
        } else {
            controls.configureNonDriveBindings()
        }

        // Set up drivetrain and auto
        com.registerAutoCommands()
        drivetrain.configurePathPlanner()
        drivetrain.registerTelemetry(logger::telemeterize)
        if (TimedRobot.isSimulation()) {
            drivetrain.seedFieldRelative(Pose2d(Translation2d(), Rotation2d.fromDegrees(90.0)))
        }

        chooser = AutoBuilder.buildAutoChooser("NONE")
        chooser.addOption("SHOOT ONLY", com.simpleShoot { ShootingSpeed.SUBWOOFER.speeds })
        SmartDashboard.putData("CHOOSE AUTO!!!", chooser)
    }

    fun updateVision() {
        vision.estimatedGlobalPose.ifPresent { est: EstimatedRobotPose ->
            // Change our trust in the measurement based on the tags we can see
            val estStdDevs = vision.estimationStdDevs
            drivetrain.addVisionMeasurement(
                est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs
            )
        }
    }

    fun simulationPeriodic() {
        vision.simulationPeriodic(drivetrain.state.Pose)

        val debugField = vision.simDebugField!!
        debugField.getObject("EstimatedRobot").pose = drivetrain.state.Pose
        debugField.getObject("EstimatedRobotModules").setPoses(drivetrain.modulePoses)
    }

}
