package frc.robot.subsystems.vision

import edu.wpi.cscore.UsbCamera
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj.DigitalOutput
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Transform2d
import frc.robot.Robot
import frc.robot.autonomous.paths.Pose2d
import frc.robot.autonomous.paths.plus
import frc.robot.subsystems.drive.DriveSubsystem
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import lib.InterpolatingTable
import lib.interpolate
import lib.runCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.epsilonEquals
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.types.Interpolatable
import org.ghrobotics.lib.vision.ToastyTargetTracker
import org.photonvision.PhotonCamera
import kotlin.math.absoluteValue
import kotlin.math.tan
import kotlin.properties.Delegates


object VisionSubsystem : FalconSubsystem() {

//    val ps3eye = ChameleonCamera("ps3eye")

//    val lifecam = ChameleonCamera("lifecam")

    val gloworm = PhotonCamera("gloworm")

    private val ledFet = DigitalOutput(7) //DigitalOutput(9).apply {
//            .apply {
//                setDirection(Relay.Direction.kForward)
//            }

    var ledsEnabled by Delegates.observable(false) { _, _, newValue ->
        ledFet.set(!newValue)
    }

    override fun lateInit() {

        gloworm.driverMode = false
        gloworm.pipelineIndex = 0


    }

    object Tracker : ToastyTargetTracker(TargetTrackerConstants(1.5.seconds, 10.feet, 100, 3)) {
        /**
         * Find the target that's closest to the robot per it's averagedPose2dRelativeToBot
         */
        fun getBestTarget() = synchronized(targets) {
            targets.asSequence()
                    .filter {
                        if (!it.isReal) return@filter false
                        val x = it.averagePose.relativeTo(DriveSubsystem.robotPosition).translation.x
                        x >= 0.0
                    }.minBy { it.averagePose.relativeTo(DriveSubsystem.robotPosition).translation.norm }
        }
    }

    override fun periodic() {
        updateTracker()

//        ledFet.set(true)
        ledsEnabled = Robot.isEnabled
    }

    /**
     * Lookup table to convert target rotation to left/right offset.
     * TODO fill this table
     */
    private val skewLUT = InterpolatingTable(
            0.0 to 0.interpolatable()
    )

    private var previousSolvePnpPose: Pose2d = Pose2d()

    private fun updateTracker() {

        if (gloworm.hasTargets()) updateTangentEstimation(Rotation2d.fromDegrees(gloworm.bestTargetPitch) + camAngle.toRotation2d(), gloworm.bestTargetYaw.degrees.toRotation2d(),
                Timer.getFPGATimestamp().seconds - gloworm.latestResult.latencyMillis.milli.seconds)

        Tracker.update()
    }

    private var lastPitch = 0.0
    private var lastYaw = 0.0
    private var yawDistanceCorrectKp = 0.006
    private var yawMultiplier = 1.06

    private fun updateTangentEstimation(pitchToHorizontal: Rotation2d, yaw: Rotation2d, timestamp: SIUnit<Second>) {
        // correct yaw in the case that Chameleon's yaw doesn't match our true yaw
        val correctedYaw = yaw * yawMultiplier

        // If we don't have new data, don't do anything.
        if (lastPitch epsilonEquals pitchToHorizontal.radians && correctedYaw.radians epsilonEquals lastYaw) return
        lastPitch = pitchToHorizontal.radians; lastYaw = correctedYaw.radians

        // from limelight we know that
        // d = (h2-h1) / tan(a1+a2)
        val heightDifferential = targetHeight - camHeight
        val distance = heightDifferential / tan(pitchToHorizontal.radians) * (1.0 + yaw.degrees.absoluteValue * yawDistanceCorrectKp)

        // A vector representing the vector between our camera and the target on the field
        val cameraToTarget = Translation2d(distance, correctedYaw)

        // Pose2d representing a rigid transform from the field to us
        val fieldToRobot = DriveSubsystem.poseBuffer[timestamp] ?: DriveSubsystem.robotPosition

        // field-oriented rotation of the target. Must either be 0 or 180 degrees.
        val goalRotation = (if (fieldToRobot.rotation.degrees + correctedYaw.degrees in -90.0..90.0)
            0.degrees else 180.degrees)
                .toRotation2d()

        Tracker.addSamples(timestamp,
                Pose2d(
                        // Transform chain (see wikipedia) that take us from the field to the target
                        // for example fieldToRobot + robotToCamera = fieldToCamera
                        fieldToRobot
                                .plus(robotToCamera)
                                .plus(Pose2d(cameraToTarget, Rotation2d()))
                                .translation, // just the translation component so that we keep our [goalRotation]
                        goalRotation
                ))
    }

    private val targetHeight = 8.feet + 2.25.inches
    private val camHeight = 17.inches // todo check
    private var camAngle = 20.degrees // 24.74.degrees + 15.degrees
    private val width = 19.625.inches * 2
    private val robotToCamera = Pose2d(Translation2d((9.5).inches, 1.25.inches), 0.degrees) // TODO adjust to cad

    @Suppress("unused")
    val bumperCamera: UsbCamera = CameraServer.getInstance().startAutomaticCapture(0).apply {
        setResolution(160, 120)
        setFPS(10)
        setName("Bumper Grabber")
        setExposureAuto()
    }!!

    @Suppress("unused")
    val intakeCamera: UsbCamera = CameraServer.getInstance().startAutomaticCapture(1).apply {
        setResolution(160, 120)
        setFPS(25)
        setName("Intake")
        setExposureAuto()
    }
}

private infix fun Pose2d.epsilonEquals(other: Pose2d) =
        this.translation.x epsilonEquals other.translation.x
                && this.translation.y epsilonEquals other.translation.y
                && this.rotation.radians epsilonEquals other.rotation.radians

private operator fun Pose2d.plus(other: Pose2d) = this.transformBy(Transform2d(other.translation, other.rotation))

inline class InterpolatingDouble(val number: Double) : Interpolatable<InterpolatingDouble> {
    override fun interpolate(endValue: InterpolatingDouble, t: Double) =
            InterpolatingDouble(number.interpolate(endValue.number, t))
}

fun Number.interpolatable() = InterpolatingDouble(toDouble())
