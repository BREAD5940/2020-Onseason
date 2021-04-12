package frc.robot.subsystems.drive

import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.wpilibj.MedianFilter
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Constants
import frc.robot.Robot
import frc.robot.autonomous.paths.plus
import frc.robot.autonomous.paths.transformBy
import frc.robot.subsystems.shooter.ShotParameter
import frc.robot.subsystems.vision.VisionSubsystem
import kotlin.math.absoluteValue
import lib.revolutionsPerMinute
import lib.toRotation2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.inRadians
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.inches

open class VisionDriveCommand : HolomonicDriveCommand() {

    init {
        SmartDashboard.putData("vision PID", controller)
    }

    private val useTracker = true

    private val angleEntry: NetworkTableEntry = SmartDashboard.getEntry("offset")

    override fun initialize() {
        angleEntry.setDefaultDouble(0.0)
        headingAveragingBuffer.reset()
    }

    private val headingAveragingBuffer = MedianFilter(5)

    override fun execute() {
        var forward = xSource() / 1.0
        var strafe = zSource() / 1.0
//        forward *= forward.absoluteValue
//        strafe *= strafe.absoluteValue

//        val shotParameter = ShotParameter(0.degrees, 0.revolutionsPerMinute, angleEntry.getDouble(0.0).degrees)

        when {
            VisionSubsystem.gloworm.hasTargets() -> {

                val speeds: ChassisSpeeds
                @Suppress("LiftReturnOrAssignment")
                val innerOrOuterGoalPose = getTargetPose()
                if (innerOrOuterGoalPose == null) {
                    super.execute()
                    return
                }
                val angle = innerOrOuterGoalPose.translation.toRotation2d()

                SmartDashboard.putNumber("Distance to target", innerOrOuterGoalPose.translation.norm) // meters

                var shotParameter = Constants.distanceLookupTable5v.get(innerOrOuterGoalPose.translation.norm) ?: ShotParameter.defaultParameter

                if (Robot.debugMode) { // override for if we're tuning
                    shotParameter = ShotParameter(0.degrees, 0.revolutionsPerMinute, angleEntry.getDouble(0.0).degrees)
                }

                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        forward, strafe, -controller.calculate(angle.radians, shotParameter.offset.inRadians()),
                        DriveSubsystem.robotPosition.rotation)

                DriveSubsystem.periodicIO.output = SwerveDriveOutput.Percent(speeds, centerOfRotation)
            }
            else -> {
                super.execute() // allow driver to rotate
            }
        }
    }

    override fun end(interrupted: Boolean) {
    }

    val lastError get() = controller.positionError

    companion object {
        val centerOfRotation = Translation2d((-4).inches, 8.inches)
        val controller = PIDController(1.7, 0.0, 0.0)

        fun getTargetPose(): Pose2d? {
//            val bestPose = VisionSubsystem.Tracker.getBestTarget()?.averagePose
            // TODO actually implement
            val bestPose = Pose2d()
            if (bestPose == null) {
                return null // todo do smth else?
            }

            val targetPose = bestPose.relativeTo(DriveSubsystem.robotPosition.plus(Pose2d(centerOfRotation, Rotation2d())))
            val shouldAimAtInnerGoal = targetPose.rotation.degrees.absoluteValue < 18

            SmartDashboard.putBoolean("shouldAimAtInnerGoal?", shouldAimAtInnerGoal)

            // decide between outer and inner goal poses to aim at

            // TODO offset by Translation between center and shooter (it's (0, 8in))

            return (if (shouldAimAtInnerGoal)
                targetPose.transformBy(Pose2d(2.feet + 5.inches, 0.inches, 0.degrees.toRotation2d()))
            else
                targetPose)
        }
    }
}
