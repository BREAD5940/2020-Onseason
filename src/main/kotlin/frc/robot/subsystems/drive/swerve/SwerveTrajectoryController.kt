package frc.robot.subsystems.drive.swerve

import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.ProfiledPIDController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.trajectory.Trajectory
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile
import frc.robot.subsystems.drive.SwerveDriveOutput
import frc.robot.subsystems.drive.toTranslation2d
import kotlin.math.PI
import lib.normalize
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.inRadians
import org.ghrobotics.lib.mathematics.units.derived.volts

class SwerveTrajectoryController(
    private val kinematics: SwerveDriveKinematics,
    private val feedforward: SimpleMotorFeedforward
) {

    private var lastTime = -1.0
    private var prevState: Array<SwerveModuleState> = arrayOf(
            SwerveModuleState(), SwerveModuleState(), SwerveModuleState(), SwerveModuleState())

    private val forwardController = PIDController(4.0, 0.0, 0.0) // x meters per second per meter of error
    private val strafeController = PIDController(4.0, 0.0, 0.0)

    private val rotationController = ProfiledPIDController(4.0, 0.0, 0.0, TrapezoidProfile.Constraints(360.degrees.inRadians(), 240.degrees.inRadians())) // rad per sec per radian of error
            .apply {
                enableContinuousInput(-PI, PI)
            }

    fun reset(heading: Rotation2d) {
        forwardController.reset()
        strafeController.reset()
        rotationController.reset(heading.radians)
    }

    fun calculate(
        time: Double,
        state: Trajectory.State,
        targetHeading: Rotation2d,
        currentPose: Pose2d
    ): SwerveDriveOutput.TrajectoryTrackerOutput {

        // dt
        if (lastTime < 0.0) lastTime = time
        val dt = time - lastTime

        // Our wanted velocity on the field
        val velocity = state.poseMeters.rotation.toTranslation2d().normalize() * state.velocityMetersPerSecond

        // P loop to convert delta in X and Y to velocity outputs, and rotation to rotations speed

        // place the output in the robot frame of reference
        // and add the trajectory velocity to it as a feedforward
        val feedbackOutput = ChassisSpeeds.fromFieldRelativeSpeeds(
                forwardController.calculate(currentPose.translation.x, state.poseMeters.translation.x) + velocity.x,
                strafeController.calculate(currentPose.translation.y, state.poseMeters.translation.y) + velocity.y,
                rotationController.calculate(currentPose.rotation.radians, targetHeading.radians),
                currentPose.rotation
        )

        // convert from chassis speeds (PID plus trajectory speeds) to states
        val states = kinematics.toSwerveModuleStates(feedbackOutput)

        // Calculate feedforwards for each module based on it's acceleration
        val outputs = arrayListOf<Mk2SwerveModule.Output.Velocity>() // Temp array
        states.forEachIndexed { index, _ ->
            val acceleration =
                    (states[index].speedMetersPerSecond - prevState[index].speedMetersPerSecond) / dt
            val moduleVelocity = states[index].speedMetersPerSecond

            val ffVoltage = feedforward.calculate(moduleVelocity, acceleration)

            outputs.add(index, Mk2SwerveModule.Output.Velocity(
                    SIUnit(moduleVelocity),
                    states[index].angle,
                    ffVoltage.volts
            ))
        }

        val maxVelocity = 11.0 /* max voltage */ / feedforward.kv
        SwerveDriveKinematics.normalizeWheelSpeeds(states, maxVelocity)

        prevState = states

        return SwerveDriveOutput.TrajectoryTrackerOutput(
                outputs[0], outputs[1], outputs[2], outputs[3]
        )
    }
}
