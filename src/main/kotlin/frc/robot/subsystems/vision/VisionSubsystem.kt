package frc.robot.subsystems.vision

import edu.wpi.cscore.UsbCamera
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj.DigitalOutput
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Transform2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Units
import frc.robot.Robot
import frc.robot.autonomous.paths.Pose2d
import frc.robot.autonomous.paths.plus
import frc.robot.subsystems.drive.DriveSubsystem
import kotlin.math.absoluteValue
import kotlin.math.tan
import kotlin.properties.Delegates
import lib.InterpolatingTable
import lib.Logger
import lib.interpolate
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.epsilonEquals
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.types.Interpolatable
import org.photonvision.PhotonCamera

object VisionSubsystem : FalconSubsystem() {

    val gloworm = PhotonCamera("gloworm")

    val logger by lazy { Logger("VisionStateSpaceLogger") }

    private val ledFet = DigitalOutput(7) // DigitalOutput(9).apply {
//            .apply {
//                setDirection(Relay.Direction.kForward)
//            }

    var ledsEnabled by Delegates.observable(false) { _, _, newValue ->
        ledFet.set(!newValue)
    }

    override fun lateInit() {

        gloworm.driverMode = false
        gloworm.pipelineIndex = 0

        logger.log("time, robot x, robot y, robot heading, overall pitch, overall yaw, field to robot, fl heading, fl speed,")
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
        val target = gloworm.latestResult

//        if (target.hasTargets()) updateTangentEstimation(Rotation2d.fromDegrees(target.bestTarget.pitch) + camAngle.toRotation2d(), -target.bestTarget.yaw.degrees.toRotation2d(),
//                Timer.getFPGATimestamp().seconds - target.latencyMillis.milli.seconds)
        if(target.hasTargets()) updateSolvePNP(target.bestTarget.cameraToTarget,
            Timer.getFPGATimestamp().seconds - target.latencyMillis.milli.seconds)
    }

    private fun updateSolvePNP(cameraToTarget: Transform2d, timestamp: SIUnit<Second>) {
        val fieldToTarget = getTarget(DriveSubsystem.robotPosition)
        val fieldToCamera = fieldToTarget.transformBy(cameraToTarget.inverse())
        val fieldToRobot = fieldToCamera.transformBy(robotToCamera.inverse())

        DriveSubsystem.addVisionPose(fieldToRobot, timestamp)
    }

    private var lastPitch = 0.0
    private var lastYaw = 0.0
    var yawDistanceCorrectKp = 0.00
    var yawMultiplier = 1.1

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
        val cameraToTargetTranslation = Translation2d(distance, correctedYaw)

        val fieldToGoal = getTarget(DriveSubsystem.robotPosition)

        // This pose maps our camera at the origin out to our target, in the robot reference frame
        // We assume we only ever see the opposing power port, and that its rotation is zero.
        val cameraToTarget =
            Transform2d(cameraToTargetTranslation, DriveSubsystem.robotPosition.rotation * -1.0 - fieldToGoal.rotation)

        // Field to camera takes us from the field origin to the camera. The inverse of cameraToTarget
        // is targetToCamera.
        val fieldToCamera: Pose2d = fieldToGoal.transformBy(cameraToTarget.inverse())

        // Field to robot is then field to camera + camera to robot
        val fieldToRobot = fieldToCamera.transformBy(robotToCamera.inverse())
        SmartDashboard.putNumber("Vision Distance", distance.inMeters())

        DriveSubsystem.addVisionPose(fieldToRobot, timestamp)

//        logger.log(Timer.getFPGATimestamp(), DriveSubsystem.robotPosition.x, DriveSubsystem.robotPosition.y, DriveSubsystem.robotPosition.rotation.degrees, pitchToHorizontal.degrees, yaw.degrees, fieldToCamera,
//            DriveSubsystem.flModule.currentState.angle.degrees, DriveSubsystem.flModule.currentState.speedMetersPerSecond,
//            DriveSubsystem.frModule.currentState.angle.degrees, DriveSubsystem.frModule.currentState.speedMetersPerSecond,
//            DriveSubsystem.brModule.currentState.angle.degrees, DriveSubsystem.brModule.currentState.speedMetersPerSecond,
//            DriveSubsystem.blModule.currentState.angle.degrees, DriveSubsystem.blModule.currentState.speedMetersPerSecond)
    }

    fun getTarget(pose: Pose2d): Pose2d {
        // If we're pointing in [-90, 90], we're pointing away from the opponent
        // so we want the power point on the "far" wall (our wall technically)
        // Otherwise, we want the closer power port
        return if(pose.rotation.degrees.absoluteValue < 90) farPowerPort
        else closePowerPort
    }

    val closePowerPort = Pose2d(0.feet, 27.feet - 94.66.inches, 180.degrees)
    val farPowerPort = Pose2d(Units.feetToMeters(54.0), Units.inchesToMeters(94.66), Rotation2d());

    val targetHeight = 8.feet + 2.25.inches
    val camHeight = 17.45.inches // todo check
    var camAngle = 25.8.degrees // 24.74.degrees + 15.degrees
    private val width = 19.625.inches * 2
    val robotToCamera = Transform2d(Translation2d((9.5).inches, 1.25.inches), 0.degrees) // TODO adjust to cad

    @Suppress("unused")
    val bumperCamera: UsbCamera? = if(RobotBase.isSimulation()) null
    else CameraServer.getInstance().startAutomaticCapture(0).apply {
        setResolution(160, 120)
        setFPS(10)
        setName("Bumper Grabber")
        setExposureAuto()
    }!!

    @Suppress("unused")
    val intakeCamera: UsbCamera? = if(RobotBase.isSimulation()) null
    else CameraServer.getInstance().startAutomaticCapture(1).apply {
        setResolution(160, 120)
        setFPS(25)
        setName("Intake")
        setExposureAuto()
    }
}

private fun Pose2d.toTransform() = Transform2d(Pose2d(), this)

private infix fun Pose2d.epsilonEquals(other: Pose2d) =
        this.translation.x epsilonEquals other.translation.x &&
                this.translation.y epsilonEquals other.translation.y &&
                this.rotation.radians epsilonEquals other.rotation.radians

private operator fun Pose2d.plus(other: Pose2d) = this.transformBy(Transform2d(other.translation, other.rotation))

inline class InterpolatingDouble(val number: Double) : Interpolatable<InterpolatingDouble> {
    override fun interpolate(endValue: InterpolatingDouble, t: Double) =
            InterpolatingDouble(number.interpolate(endValue.number, t))
}

fun Number.interpolatable() = InterpolatingDouble(toDouble())

private fun Transform2d(translation2d: Translation2d, degrees: SIUnit<Radian>) = Transform2d(translation2d, degrees.toRotation2d())

