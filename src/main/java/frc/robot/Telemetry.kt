package frc.robot

import com.ctre.phoenix6.Utils
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit

/**
 * Telemetry; this class was copied approximately unchanged from the CTRE SwerveWithPathPlanner
 * example
 */
class Telemetry
/**
 * Construct a telemetry object, with the specified max speed of the robot
 *
 * @param maxSpeed Maximum speed in meters per second
 */(private val maxSpeed: Double) {
    /* What to publish over networktables for telemetry */
    private val inst = NetworkTableInstance.getDefault()

    /* Robot pose for field positioning */
    private val table = inst.getTable("Pose")
    private val fieldPub = table.getDoubleArrayTopic("robotPose").publish()
    private val fieldTypePub = table.getStringTopic(".type").publish()

    /* Robot speeds for general checking */
    private val driveStats = inst.getTable("Drive")
    private val velocityX = driveStats.getDoubleTopic("Velocity X").publish()
    private val velocityY = driveStats.getDoubleTopic("Velocity Y").publish()
    private val speed = driveStats.getDoubleTopic("Speed").publish()
    private val odomPeriod = driveStats.getDoubleTopic("Odometry Period").publish()

    /* Keep a reference of the last pose to calculate the speeds */
    private var lastPose = Pose2d()
    private var lastTime = Utils.getCurrentTimeSeconds()

    /* Mechanisms to represent the swerve module states */
    private val moduleMechanisms = arrayOf(
        Mechanism2d(1.0, 1.0), Mechanism2d(1.0, 1.0), Mechanism2d(1.0, 1.0), Mechanism2d(1.0, 1.0),
    )

    /* A direction and length changing ligament for speed representation */
    private val moduleSpeeds = arrayOf(
        moduleMechanisms[0]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(MechanismLigament2d("Speed", 0.5, 0.0)),
        moduleMechanisms[1]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(MechanismLigament2d("Speed", 0.5, 0.0)),
        moduleMechanisms[2]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(MechanismLigament2d("Speed", 0.5, 0.0)),
        moduleMechanisms[3]
            .getRoot("RootSpeed", 0.5, 0.5)
            .append(MechanismLigament2d("Speed", 0.5, 0.0)),
    )

    /* A direction changing and length constant ligament for module direction */
    private val m_moduleDirections = arrayOf(
        moduleMechanisms[0]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(MechanismLigament2d("Direction", 0.1, 0.0, 0.0, Color8Bit(Color.kWhite))),
        moduleMechanisms[1]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(MechanismLigament2d("Direction", 0.1, 0.0, 0.0, Color8Bit(Color.kWhite))),
        moduleMechanisms[2]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(MechanismLigament2d("Direction", 0.1, 0.0, 0.0, Color8Bit(Color.kWhite))),
        moduleMechanisms[3]
            .getRoot("RootDirection", 0.5, 0.5)
            .append(MechanismLigament2d("Direction", 0.1, 0.0, 0.0, Color8Bit(Color.kWhite))),
    )

    /* Accept the swerve drive state and telemeterize it to smartdashboard */
    fun telemeterize(state: SwerveDriveState) {
        /* Telemeterize the pose */
        val pose = state.Pose
        fieldTypePub.set("Field2d")
        fieldPub.set(doubleArrayOf(pose.x, pose.y, pose.rotation.degrees))

        /* Telemeterize the robot's general speeds */
        val currentTime = Utils.getCurrentTimeSeconds()
        val diffTime = currentTime - lastTime
        lastTime = currentTime
        val distanceDiff = pose.minus(lastPose).translation
        lastPose = pose

        val velocities = distanceDiff.div(diffTime)

        speed.set(velocities.norm)
        velocityX.set(velocities.x)
        velocityY.set(velocities.y)
        odomPeriod.set(state.OdometryPeriod)

        /* Telemeterize the module's states */
        for (i in 0..3) {
            moduleSpeeds[i].setAngle(state.ModuleStates[i].angle)
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle)
            moduleSpeeds[i].length =
                state.ModuleStates[i].speedMetersPerSecond / (2 * maxSpeed)

            SmartDashboard.putData("Module $i", moduleMechanisms[i])
        }


        quest.putData()
    }

    val quest = QuestJavaInteraction()
}
