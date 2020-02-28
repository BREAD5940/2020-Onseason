package frc.robot.subsystems.vision

import edu.wpi.first.wpilibj.Relay
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Transform2d
import edu.wpi.first.wpilibj.geometry.Pose2d
import frc.robot.autonomous.paths.Pose2d
import frc.robot.autonomous.paths.plus
import frc.robot.autonomous.paths.transformBy
import frc.robot.subsystems.drive.DriveSubsystem
import kotlin.math.pow
import kotlin.math.sqrt
import lib.InterpolatingTable
import lib.interpolate
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.epsilonEquals
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.seconds
import org.ghrobotics.lib.types.Interpolatable
import org.ghrobotics.lib.vision.ChameleonCamera
import org.ghrobotics.lib.vision.ToastyTargetTracker
import kotlin.math.tan
import kotlin.properties.Delegates

object VisionSubsystem : FalconSubsystem() {

//    val ps3eye = ChameleonCamera("ps3eye")

    val lifecam = ChameleonCamera("lifecam")

    private val ledFet = Relay(0) //DigitalOutput(9).apply {
            .apply {
                setDirection(Relay.Direction.kForward)
            }

    var ledsEnabled by Delegates.observable(false) {
        _, _, newValue -> ledFet.set(if(newValue) Relay.Value.kForward else Relay.Value.kOff)
    }

    override fun lateInit() {
        lifecam.driverMode = false
        lifecam.pipeline = 1.0
    }

    object Tracker : ToastyTargetTracker(TargetTrackerConstants(2.0.seconds, 10.feet, 100, 10)) {
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
        ledFet.set(Relay.Value.kForward)
//        ledsEnabled = Robot.isEnabled
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

        if(lifecam.isValid) updateTangentEstimation(lifecam.pitch + camAngle.toRotation2d(), lifecam.yaw,
                Timer.getFPGATimestamp().seconds - lifecam.latency)

//        updateSolvePNP()

        Tracker.update()

        //        val d = (targetHeight - camHeight) / tan(lifecam.pitch.radians + camAngle.inRadians())

//        val w = lifecam.minRectWidth
//        val h = lifecam.minRectHeight
//        val width_ = if (w > h) w else h
//
//        val d = sqrt((width.inMeters() * focalLen / (width_)).pow(2) -
//                (targetHeight - camHeight).inMeters().pow(2)).meters
//
//        val yaw = lifecam.yaw
//
//        var skew = if (lifecam.minRectHeight > lifecam.minRectWidth) lifecam.minRectSkew + 90.degrees else lifecam.minRectSkew
//
//        while (skew < (-180).degrees) skew += 180.degrees
//        while (skew > 180.degrees) skew -= 180.degrees
//
//        val offset = skewLUT.get(skew.inDegrees())?.number ?: 0.0
//
//        val pose = DriveSubsystem.robotPosition.plus(Transform2d(
//                Translation2d(d, yaw), Rotation2d.fromDegrees(offset)
//        ))
    }

    private fun updateTangentEstimation(pitchToHorizontal: Rotation2d, yaw: Rotation2d, timestamp: SIUnit<Second>) {
        // from limelight
        // d = (h2-h1) / tan(a1+a2)

        val heightDifferential = targetHeight - camHeight
        val distance = heightDifferential / tan(pitchToHorizontal.radians)

        val cameraToTarget = Translation2d(distance, yaw)
        val fieldToRobot = DriveSubsystem.poseBuffer[timestamp] ?: DriveSubsystem.robotPosition

        val goalRotation = (if(fieldToRobot.rotation.degrees + yaw.degrees in -90.0..90.0)
            0.degrees else 180.degrees)
                .toRotation2d()

        Tracker.addSamples(timestamp,
                Pose2d(
                        fieldToRobot
                                .plus(robotToCamera)
                                .plus(Pose2d(cameraToTarget, Rotation2d()))
                                .translation,
                        goalRotation
                ))
    }

    private fun updateSolvePNP() {
        // check that the pose is valid -- if not, it defaults to 0.0 for x, y, and rotation
        val solvePnpPose = lifecam.bestPose

        if(previousSolvePnpPose epsilonEquals solvePnpPose) return
        previousSolvePnpPose = solvePnpPose

        if (solvePnpPose.translation.x epsilonEquals 0.0 && solvePnpPose.translation.y epsilonEquals 0.0 &&
                solvePnpPose.rotation.radians epsilonEquals 0.0) return

        val drivetrainPose = DriveSubsystem.poseBuffer[Timer.getFPGATimestamp().seconds - lifecam.latency] ?: DriveSubsystem.robotPosition

        val fieldRelativePose = drivetrainPose
                .transformBy(robotToCamera)// transform by camera position
                .transformBy(solvePnpPose) // transform camera pos by measured pose

        Tracker.addSamples(Timer.getFPGATimestamp().seconds - lifecam.latency,
                listOf(fieldRelativePose))
    }

    private val targetHeight = 8.feet + 2.25.inches
    private val camHeight = 13.inches // todo check
    private val camAngle = 24.74.degrees
    private val width = 19.625.inches * 2
    private val focalLen = (44 /* px */ * sqrt(10.feet.inMeters().pow(2) +
            targetHeight.inMeters().pow(2))) / (width.inMeters()) / 0.35
    private val robotToCamera = Pose2d(Translation2d((-0.5).inches, 3.inches), 0.degrees) // TODO adjust to cad
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
