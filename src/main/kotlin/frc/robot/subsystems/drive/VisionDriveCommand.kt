package frc.robot.subsystems.drive

import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.wpilibj.MedianFilter
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.autonomous.paths.transformBy
import frc.robot.subsystems.shooter.FlywheelSubsystem
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
import org.ghrobotics.lib.mathematics.units.meters

class VisionDriveCommand : HolomonicDriveCommand() {

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
        var forward = -xSource() / 1.0
        var strafe = -zSource() / 1.0
        forward *= forward.absoluteValue
        strafe *= strafe.absoluteValue

        val shotParameter = FlywheelSubsystem.defaultShotLookupTable.get(VisionSubsystem.ps3eye.pitch.degrees) ?: ShotParameter.DefaultParameter
//        val shotParameter = ShotParameter(0.degrees, 0.revolutionsPerMinute, angleEntry.getDouble(0.0).degrees)

        when {
            VisionSubsystem.ps3eye.isValid -> {

                val speeds: ChassisSpeeds
                @Suppress("ConstantConditionIf", "LiftReturnOrAssignment")
                if (useTracker) {

                    val bestPose = VisionSubsystem.Tracker.getBestTarget()?.averagePose
                    if (bestPose == null) {
                        super.execute()
                        return // todo do smth else?
                    }
                    val angle = bestPose.relativeTo(DriveSubsystem.robotPosition)
                            .transformBy(Pose2d(2.feet + 5.inches, 0.inches, 0.degrees.toRotation2d())) // offset to get inner port
                            .translation.toRotation2d()

                    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                            forward, strafe, controller.calculate(angle.radians, shotParameter.offset.inRadians()),
                            DriveSubsystem.robotPosition.rotation)
                } else {
                    val avHeading = headingAveragingBuffer.calculate(VisionSubsystem.ps3eye.yaw.radians + DriveSubsystem.robotPosition.rotation.radians)

                    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                            forward, strafe, controller.calculate(DriveSubsystem.robotPosition.rotation.radians,
                            avHeading + shotParameter.offset.inRadians()),
                            DriveSubsystem.robotPosition.rotation)
                }

                DriveSubsystem.periodicIO.output = SwerveDriveOutput.Percent(speeds, centerOfRotation)
            }
            else -> {
                super.execute()
            }
        }
    }

    companion object {
        val centerOfRotation = Translation2d(0.meters, 8.inches)
        val controller = PIDController(2.8, 0.0, 0.3)

        val rightBelowGoalParameter = ShotParameter(44.degrees, 1600.revolutionsPerMinute)
    }
}
