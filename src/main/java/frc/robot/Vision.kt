// Borrowed from PhotonLib examples
// Note: when PhotonLib up dates the json will need to be updated
package frc.robot

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import org.photonvision.EstimatedRobotPose
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.PhotonPoseEstimator.PoseStrategy
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import org.photonvision.simulation.VisionSystemSim
import org.photonvision.targeting.PhotonTrackedTarget
import java.util.*
import kotlin.math.abs

// TODO post vision poses to dashboard
class Vision {
    private val camera: PhotonCamera = PhotonCamera(CAMERA_NAME)
    private val photonEstimator: PhotonPoseEstimator =
        PhotonPoseEstimator(
            kTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            ROBOT_TO_CAM
        ).apply {
            setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY)
        }

    /**
     * Returns the latest standard deviations of the estimated pose from [ ][.getEstimatedGlobalPose], for use with [ ]. This should
     * only be used when there are targets visible.
     */
    var estimationStdDevs: Matrix<N3, N1>? = null

    // Simulation
    private var cameraSim: PhotonCameraSim? = null
    private var visionSim: VisionSystemSim? = null

    init {
        // ----- Simulation
        if (TimedRobot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = VisionSystemSim("main")
            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            visionSim!!.addAprilTags(kTagLayout)
            // Create simulated camera properties. These can be set to mimic your actual camera.
            // TODO: these need to be correctly set to mimic the actual camera
            val cameraProp = SimCameraProperties()
            cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(90.0))
            cameraProp.setCalibError(0.35, 0.10)
            cameraProp.fps = 15.0
            cameraProp.avgLatencyMs = 50.0
            cameraProp.latencyStdDevMs = 15.0
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
            // targets.
            cameraSim = PhotonCameraSim(camera, cameraProp)
            // Add the simulated camera to view the targets on this simulated field.
            visionSim!!.addCamera(cameraSim, ROBOT_TO_CAM)

            cameraSim!!.enableDrawWireframe(true)
        }
    }

    val estimatedGlobalPose: Optional<EstimatedRobotPose>
        /**
         * The latest estimated robot pose on the field from vision data. This may be empty. This should
         * only be called once per loop.
         *
         *
         * Also includes updates for the standard deviations, which can (optionally) be retrieved with
         * getEstimationStdDevs
         *
         * @return An [EstimatedRobotPose] with an estimated pose, estimate timestamp, and targets
         * used for estimation.
         */
        get() {
            var visionEst = Optional.empty<EstimatedRobotPose>()
            for (change in camera.allUnreadResults) {
                visionEst = photonEstimator.update(change)
                updateEstimationStdDevs(visionEst, change.getTargets())

                if (TimedRobot.isSimulation()) {
                    visionEst.ifPresentOrElse(
                        { est: EstimatedRobotPose ->
                            simDebugField!!.getObject("VisionEstimation").pose =
                                est.estimatedPose.toPose2d()
                        },
                        {
                            simDebugField!!.getObject("VisionEstimation").setPoses()
                        })
                }
            }

            // Filter out obviously incorrect poses
            var usePose = true
            if (visionEst.isPresent) {
                val pose3d = visionEst.get().estimatedPose

                if (abs(pose3d.z) > 0.1) {
                    usePose = false
                }
                if (pose3d.y < 0 || pose3d.x < 0) {
                    usePose = false
                }
                if (pose3d.y > 8.21 || pose3d.x > 16.54) {
                    usePose = false
                }
            }

            if (!usePose) {
                visionEst = Optional.empty()
            }
            return visionEst
        }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private fun updateEstimationStdDevs(
        estimatedPose: Optional<EstimatedRobotPose>, targets: List<PhotonTrackedTarget>
    ) {
        if (estimatedPose.isEmpty) {
            // No pose input. Default to single-tag std devs
            estimationStdDevs = kSingleTagStdDevs
        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs
            var numTags = 0
            var avgDist = 0.0

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            targets.forEach {
                val tagPose = photonEstimator.fieldTags.getTagPose(it.getFiducialId())
                if (tagPose.isEmpty) {
                    return
                }
                numTags++
                avgDist +=
                    tagPose
                        .get()
                        .toPose2d()
                        .translation
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().translation)
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                estimationStdDevs = kSingleTagStdDevs
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags.toDouble()
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) {
                    estStdDevs = kMultiTagStdDevs
                }
                // Increase std devs based on (average) distance
                estStdDevs = if (numTags == 1 && avgDist > 4) {
                    VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE)
                } else {
                    estStdDevs.times(1 + (avgDist * avgDist / 30))
                }
                estimationStdDevs = estStdDevs
            }
        }
    }

    // ----- Simulation
    fun simulationPeriodic(robotSimPose: Pose2d?) {
        visionSim!!.update(robotSimPose)
    }

    /** Reset pose history of the robot in the vision system simulation.  */
    fun resetSimPose(pose: Pose2d?) {
        if (TimedRobot.isSimulation()) {
            visionSim!!.resetRobotPose(pose)
        }
    }

    val simDebugField: Field2d?
        /** A Field2d for visualizing our robot and objects on the field.  */
        get() =
            if (TimedRobot.isSimulation()) visionSim!!.debugField
            else null


    companion object {
        const val CAMERA_NAME = "Arducam_OV9281_USB_Camera"

        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        val ROBOT_TO_CAM = Transform3d(
            Translation3d(
                Units.inchesToMeters(3.3125),
                0.0,
                Units.inchesToMeters(11.75)
            ),  // TODO approximation, fix later
            Rotation3d(0.0, Math.toRadians(-30.0), 0.0)
        )

        // The layout of the AprilTags on the field
        val kTagLayout: AprilTagFieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField()

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.) TODO
        val kSingleTagStdDevs: Matrix<N3, N1> = VecBuilder.fill(4.0, 4.0, 8.0)
        val kMultiTagStdDevs: Matrix<N3, N1> = VecBuilder.fill(0.5, 0.5, 1.0)
    }
}
