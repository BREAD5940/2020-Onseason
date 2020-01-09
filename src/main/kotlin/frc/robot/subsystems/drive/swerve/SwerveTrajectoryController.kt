package frc.robot.subsystems.drive.swerve

import  edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.trajectory.Trajectory
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.drive.SwerveDriveOutput
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.inRadians
import org.ghrobotics.lib.utils.safeRangeTo

class SwerveTrajectoryController(
    private val kinematics: SwerveDriveKinematics,
    private val feedforward: SimpleMotorFeedforward
) {

    private var lastTime = -1.0
//    private var prevState = listOf(
//            SwerveModuleState(), SwerveModuleState(), SwerveModuleState(), SwerveModuleState())

    private val forwardController = PIDController(10.0, 0.0, 0.0) // x meters per second per meter of error
    private val strafeController = PIDController(10.0, 0.0, 0.0)
    private val rotationController = PIDController(2.0, 0.0, 0.0) // rad per sec per radian of error

    private val maxRotRange = (-45.degrees.inRadians()..45.degrees.inRadians())

    fun calculate(
            time: Double,
            desiredState: Trajectory.State,
            targetHeading: Rotation2d,
            currentPose: Pose2d
    ): SwerveDriveOutput.TrajectoryTrackerOutput {

        // dt
        if (lastTime < 0.0) lastTime = time
        val dt = time - lastTime
        val desiredPose = desiredState.poseMeters

        val poseError: Pose2d = desiredPose.relativeTo(currentPose)

        var targetXVel: Double = forwardController.calculate(
                currentPose.translation.x,
                desiredPose.translation.x)

        var targetYVel: Double = strafeController.calculate(
                currentPose.translation.y,
                desiredPose.translation.y)

        // The robot will go to the desired rotation of the final pose in the trajectory,
        // not following the poses at individual states.
        // The robot will go to the desired rotation of the final pose in the trajectory,
        // not following the poses at individual states.
        val targetAngularVel: Double = rotationController.calculate(
                currentPose.rotation.radians,
                targetHeading.radians).coerceIn(maxRotRange)

        val vRef: Double = desiredState.velocityMetersPerSecond

        targetXVel += vRef * poseError.rotation.cos
        targetYVel += vRef * poseError.rotation.sin

        val targetChassisSpeeds = ChassisSpeeds(targetXVel, targetYVel, targetAngularVel)

        val targetModuleStates: Array<SwerveModuleState> = DriveSubsystem.kinematics.toSwerveModuleStates(targetChassisSpeeds)

        return SwerveDriveOutput.TrajectoryTrackerOutput(targetModuleStates)
    }
}
