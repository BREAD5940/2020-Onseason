package frc.robot.subsystems

import edu.wpi.first.wpilibj.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Transform2d
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.util.Units
import edu.wpi.first.wpiutil.math.VecBuilder
import frc.robot.Constants
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.vision.VisionSubsystem
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.inRadians
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.junit.Test
import org.opencv.core.Point
import org.photonvision.PhotonUtils
import java.io.FileReader
import kotlin.math.absoluteValue
import kotlin.math.tan

class VisionTest {
    @Test
    fun testOldData() {
        val file = FileReader("log/robotvisiondata.csv")

        val startingAngle = 178.3599854.degrees.toRotation2d()
        val odometry = SwerveDrivePoseEstimator(
            startingAngle, Pose2d(9.0, 3.3, startingAngle), Constants.kinematics,
            VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5.0)),
            VecBuilder.fill(Units.degreesToRadians(1.0)),
            VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10.0)),
            1.0 / 50.0
        )

        odometry.resetPosition(Pose2d(9.0, 3.3, startingAngle), startingAngle)

        val lines: List<List<Double>> = file.readLines().map { it.split(",") }.let { it.subList(1, it.size) }
            .map { it.map { it1 -> it1.toDouble() } }
        lines.forEach {
            val time = it[0]

            /** Module speeds as (fl, fr, br, bl) */
            odometry.updateWithTime(
                time,
                it[3].degrees.toRotation2d(),
                SwerveModuleState(it[7], it[6].degrees.toRotation2d()),
                SwerveModuleState(it[9], it[8].degrees.toRotation2d()),
                SwerveModuleState(it[11], it[10].degrees.toRotation2d()),
                SwerveModuleState(it[13], it[12].degrees.toRotation2d())
            )

            val visionPitch = it[4]
            val visionYaw = it[5]

            val pose = updateTangentEstimation(visionPitch.degrees.toRotation2d(), visionYaw.degrees.toRotation2d())
            odometry.addVisionMeasurement(pose, time - 10.milli.seconds.inSeconds())

            println(odometry.estimatedPosition.tocsv())
        }
    }


    private fun updateTangentEstimation(pitchToHorizontal: Rotation2d, yaw: Rotation2d): Pose2d {
        // correct yaw in the case that Chameleon's yaw doesn't match our true yaw
        val correctedYaw = yaw * VisionSubsystem.yawMultiplier

        // from limelight we know that
        // d = (h2-h1) / tan(a1+a2)

        val heightDifferential = VisionSubsystem.targetHeight - VisionSubsystem.camHeight
        val distance =
            heightDifferential / tan(pitchToHorizontal.radians) * (1.0 + yaw.degrees.absoluteValue * VisionSubsystem.yawDistanceCorrectKp)

        // A vector representing the vector between our camera and the target on the field
        val cameraToTargetTranslation = Translation2d(distance, correctedYaw)

        // This pose maps our camera at the origin out to our target, in the robot reference frame
        // We assume we only ever see the opposing power port, and that its rotation is zero.
        val cameraToTarget =
            Transform2d(cameraToTargetTranslation, DriveSubsystem.robotPosition.rotation * -1.0)

        // Field to camera takes us from the field origin to the camera. The inverse of cameraToTarget
        // is targetToCamera.
        val fieldToCamera: Pose2d =
            VisionSubsystem.getTarget(DriveSubsystem.robotPosition).transformBy(cameraToTarget.inverse())

        // Field to robot is then field to camera + camera to robot
        val fieldToRobot = fieldToCamera.transformBy(VisionSubsystem.robotToCamera.inverse())

        return fieldToRobot

//        DriveSubsystem.addVisionPose(fieldToRobot, timestamp)
//        VisionSubsystem.logger.log(
//            Timer.getFPGATimestamp(), DriveSubsystem.robotPosition.x, DriveSubsystem.robotPosition.y, DriveSubsystem.robotPosition.rotation.degrees, pitchToHorizontal.degrees, yaw.degrees, fieldToCamera,
//            DriveSubsystem.flModule.currentState.angle.degrees, DriveSubsystem.flModule.currentState.speedMetersPerSecond,
//            DriveSubsystem.frModule.currentState.angle.degrees, DriveSubsystem.frModule.currentState.speedMetersPerSecond,
//            DriveSubsystem.brModule.currentState.angle.degrees, DriveSubsystem.brModule.currentState.speedMetersPerSecond,
//            DriveSubsystem.blModule.currentState.angle.degrees, DriveSubsystem.blModule.currentState.speedMetersPerSecond)
    }


    @Test
    fun testDistance() {

//        val pitch = .degrees
//        val dist = 10.feet

//        val pitch = 0.degrees
//        val dist = 167.inches

//        val pitch = 22.degrees
//        val dist = 75.inches

        val pitch = 8.23.degrees
        val dist = 121.inches

        val photondist = PhotonUtils.calculateDistanceToTargetMeters(
            VisionSubsystem.camHeight.inMeters(), VisionSubsystem.targetHeight.inMeters(),
            VisionSubsystem.camAngle.inRadians(), pitch.inRadians()
        ).meters
        println("Got dist: ${photondist.inInches()}, expected ${dist.inInches()}")
    }
}

private fun Pose2d.tocsv() = "${this.x}, ${this.y}, ${this.rotation.degrees}"
