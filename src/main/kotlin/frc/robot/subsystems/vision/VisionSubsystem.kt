package frc.robot.subsystems.vision

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Transform2d
import edu.wpi.first.wpilibj.geometry.Pose2d
import frc.robot.autonomous.paths.Pose2d
import frc.robot.autonomous.paths.plus
import frc.robot.autonomous.paths.transformBy
import frc.robot.subsystems.drive.DriveSubsystem
import lib.FlippableDIOPin
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
import kotlin.math.absoluteValue
import kotlin.math.tan
import kotlin.properties.Delegates

object VisionSubsystem : FalconSubsystem() {

//    val ps3eye = ChameleonCamera("ps3eye")

    val lifecam = ChameleonCamera("lifecam")

    private val ledFet = FlippableDIOPin(9) //DigitalOutput(9).apply {
//            .apply {
//                setDirection(Relay.Direction.kForward)
//            }

    var ledsEnabled by Delegates.observable(false) {
        _, _, newValue -> ledFet.set(newValue)

    }

    override fun lateInit() {
        lifecam.driverMode = false
        lifecam.pipeline = 1.0
    }

    object Tracker : ToastyTargetTracker(TargetTrackerConstants(0.8.seconds, 10.feet, 100, 4)) {
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

//        DriveSubsystem.odometry.resetPosition(Pose2d(3.0, 3.0, 180.degrees.toRotation2d()), DriveSubsystem.gyro())

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

    private var lastPitch = 0.0
    private var lastYaw = 0.0
    private var yawDistanceCorrectKp = 0.006
    private var yawMultiplier = 1.06

    private fun updateTangentEstimation(pitchToHorizontal: Rotation2d, yaw: Rotation2d, timestamp: SIUnit<Second>) {
        // from limelight
        // d = (h2-h1) / tan(a1+a2)
        val correctedYaw = yaw * yawMultiplier

        if(lastPitch epsilonEquals pitchToHorizontal.radians && correctedYaw.radians epsilonEquals lastYaw) return
        lastPitch = pitchToHorizontal.radians; lastYaw = correctedYaw.radians

        val heightDifferential = targetHeight - camHeight
        val distance = heightDifferential / tan(pitchToHorizontal.radians) * (1.0 + yaw.degrees.absoluteValue * yawDistanceCorrectKp)

        val cameraToTarget = Translation2d(distance, correctedYaw)
        val fieldToRobot = DriveSubsystem.poseBuffer[timestamp] ?: DriveSubsystem.robotPosition

//        println("DISTANCE TO TARGET ${cameraToTarget.norm.meters.inFeet()} AT ANGLE ${correctedYaw.degrees}")

        val goalRotation = (if(fieldToRobot.rotation.degrees + correctedYaw.degrees in -90.0..90.0)
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
    private val camHeight = 17.inches // todo check
    private var camAngle = 20.degrees // 24.74.degrees + 15.degrees
    private val width = 19.625.inches * 2
    private val focalLen = (44 /* px */ * sqrt(10.feet.inMeters().pow(2) +
            targetHeight.inMeters().pow(2))) / (width.inMeters()) / 0.35
    private val robotToCamera = Pose2d(Translation2d((9.5).inches, 1.25.inches), 0.degrees) // TODO adjust to cad
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
