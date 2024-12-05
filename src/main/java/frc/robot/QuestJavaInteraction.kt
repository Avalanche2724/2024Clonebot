package frc.robot

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Quaternion
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.networktables.*
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard


class QuestJavaInteraction {
    var nt4Instance: NetworkTableInstance = NetworkTableInstance.getDefault()
    var nt4Table: NetworkTable = nt4Instance.getTable("oculus")
    private val questMiso: IntegerSubscriber = nt4Table.getIntegerTopic("miso").subscribe(0)
    private val questMosi: IntegerPublisher = nt4Table.getIntegerTopic("mosi").publish()

    // Subscribe to the Network Tables oculus data topics
    private val questFrameCount: IntegerSubscriber = nt4Table.getIntegerTopic("frameCount").subscribe(0)
    private val questTimestamp: DoubleSubscriber = nt4Table.getDoubleTopic("timestamp").subscribe(0.0)
    private val questPosition: FloatArraySubscriber =
        nt4Table.getFloatArrayTopic("position").subscribe(floatArrayOf(0.0f, 0.0f, 0.0f))
    private val questQuaternion: FloatArraySubscriber =
        nt4Table.getFloatArrayTopic("quaternion").subscribe(floatArrayOf(0.0f, 0.0f, 0.0f, 0.0f))
    private val questEulerAngles: FloatArraySubscriber =
        nt4Table.getFloatArrayTopic("eulerAngles").subscribe(floatArrayOf(0.0f, 0.0f, 0.0f))

    private val resetPosition = Pose2d(Translation2d(0.0, Rotation2d.fromDegrees(0.0)), Rotation2d.fromDegrees(0.0))

    private val yaw_offset = 0.0f

    fun putData() {
        val pose = getOculusPose().minus(resetPosition)
        SmartDashboard.putNumberArray("OculusAll",
            doubleArrayOf(
                pose.x, pose.y, pose.rotation.degrees
            )
        )
    }


    // Zero the absolute 3D position of the robot (similar to long-pressing the quest logo)
    fun zeroPosition() {
        //resetOdometry(Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0)))
        if (questMiso.get() != 99L) {
            questMosi.set(1)
        }
    }

    // Clean up oculus subroutine messages after processing on the headset
    fun cleanUpOculusMessages() {
        if (questMiso.get() == 99L) {
            questMosi.set(0)
        }
    }

    private fun getOculusYaw(): Float {
        val eulerAngles = questEulerAngles.get()
        return eulerAngles[1] - yaw_offset
    }

    private fun getOculusPosition(): Translation2d {
        val oculusPosition = questPosition.get()
        return Translation2d(oculusPosition[2].toDouble(), -oculusPosition[0].toDouble())
    }

    private fun getOculusPose(): Pose2d {
        return Pose2d(getOculusPosition(), Rotation2d.fromDegrees(getOculusYaw().toDouble()))
    }

}