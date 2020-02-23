package frc.robot.subsystems.vision

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Transform2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
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
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.meters
import org.ghrobotics.lib.mathematics.units.seconds
import org.ghrobotics.lib.types.Interpolatable
import org.ghrobotics.lib.vision.ChameleonCamera
import org.ghrobotics.lib.vision.TargetTracker
import org.ghrobotics.lib.vision.ToastyTargetTracker

object VisionSubsystem : FalconSubsystem() {

//    val lifecam = ChameleonCamera("lifecam") // TODO find actual name

    val ps3eye = ChameleonCamera("ps3eye")

    override fun lateInit() {
        ps3eye.driverMode = false
        ps3eye.pipeline = 1.0
    }

    object Tracker : ToastyTargetTracker(TargetTrackerConstants(1.0.seconds, 10.feet, 50, 10)) {
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
        if (!ps3eye.isValid) return

        // check that the pose is valid -- if not, it defaults to 0.0 for x, y, and rotation
        val solvePnpPose = ps3eye.bestPose

        if(previousSolvePnpPose epsilonEquals solvePnpPose) return
        previousSolvePnpPose = solvePnpPose

        if (solvePnpPose.translation.x epsilonEquals 0.0 && solvePnpPose.translation.y epsilonEquals 0.0 &&
                solvePnpPose.rotation.radians epsilonEquals 0.0) return

        val drivetrainPose = DriveSubsystem.poseBuffer[Timer.getFPGATimestamp().seconds - ps3eye.latency] ?: DriveSubsystem.robotPosition

        val fieldRelativePose = drivetrainPose
                .transformBy(kCameraPos)// transform by camera position
                .transformBy(solvePnpPose) // transform camera pos by measured pose

        Tracker.addSamples(Timer.getFPGATimestamp().seconds - ps3eye.latency,
                listOf(fieldRelativePose))

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

    private val targetHeight = 9.feet + 9.inches + 1.feet + 5.inches
    private val camHeight = 13.inches // todo check
    private val camAngle = 47.degrees
    private val width = 19.625.inches * 2
    private val focalLen = (44 /* px */ * sqrt(10.feet.inMeters().pow(2) +
            targetHeight.inMeters().pow(2))) / (width.inMeters()) / 0.35
    private val kCameraPos = Pose2d(Translation2d((-0.5).inches, 3.inches), Rotation2d())
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
