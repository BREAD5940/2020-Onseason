package frc.robot.auto.paths

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.trajectory.Trajectory
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint
import frc.robot.Constants
import frc.robot.autonomous.paths.TrajectoryWaypoints
import frc.robot.subsystems.drive.DriveSubsystem.kinematics
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.inch
import kotlin.math.max

object TrajectoryFactory {

    /** Constraints **/

    val kMaxVelocity = 7.5.feet.velocity
    val kMaxAcceleration = 6.feet.acceleration

    private val kMaxHabitatVelocity = 3.5.feet.velocity

    private val kFirstPathMaxAcceleration = 6.feet.acceleration

    private val kVelocityRadiusConstraintRadius = 4.5.feet
    private val kVelocityRadiusConstraintVelocity = 3.75.feet.velocity

    private val kMaxCentripetalAccelerationElevatorUp = 6.feet.acceleration
    private val kMaxCentripetalAccelerationElevatorDown = 7.5.feet.acceleration

    val kMaxVoltage = 10.volt

    /** Adjusted Poses **/


    /** Trajectories **/


    val testTrajectory2 = generateTrajectory(
            false,
            listOf(
                    Pose2d(5.396.feet, 5.551.feet, 0.degree).asWaypoint(),
                    Pose2d(10.004.feet, 5.648.feet, 0.degree).asWaypoint(),
                    Pose2d(8.334.feet, 2.474.feet, (-145.637).degree).asWaypoint(),
                    Pose2d(7.102.feet, 6.796.feet, 71.162.degree).asWaypoint()
            ),
            getConstraints(true, Pose2d()), 5.feet.velocity, kMaxAcceleration, kMaxVoltage
    )


    fun Pose2d.withRotation(rotation: SIUnit<Radian>) = Pose2d(this.translation, rotation.toRotation2d())
    fun TrajectoryWaypoints.Waypoint.withRotation(rotation: SIUnit<Radian>) = Pose2d(this.position.translation, rotation.toRotation2d()).asWaypoint()

    val testTrajectory by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(1.5.feet, 23.feet, 0.degree).asWaypoint(),
                        Pose2d(11.5.feet, 23.feet, 0.degree).asWaypoint()
                ),
                getConstraints(maxSpeedMetersPerSecond = kMaxVelocity.value), kMaxVelocity, 7.feet.acceleration
        )
    }

    /** Generation **/

    private fun getConstraints(
            maxSpeedMetersPerSecond: Double
    ) =
            listOf(
                SwerveDriveKinematicsConstraint(kinematics, maxSpeedMetersPerSecond)
            )

   /* fun getConstraints(
            maxSpeedMetersPerSecond: Double
    ) =
            getConstraints(maxSpeedMetersPerSecond)*/

    fun generateTrajectory(
            reversed: Boolean,
            points: List<TrajectoryWaypoints.Waypoint>,
            constraints: List<TrajectoryConstraint>,
            maxVelocity: SIUnit<Velocity<Meter>>,
            maxAcceleration: SIUnit<Acceleration<Meter>>,
            //maxVoltage: SIUnit<Volt>,
            //optimizeCurvature: Boolean = true,
            endVelocity: SIUnit<Velocity<Meter>> = 0.inch.velocity
    ): Trajectory? {

        //val driveDynamicsConstraint = SwerveDriveKinematicsConstraint(Constants.DriveConstants.kHighGearDifferentialDrive, maxVoltage)
        val allConstraints = ArrayList<TrajectoryConstraint>()

        //allConstraints.add(driveDynamicsConstraint)
        if (constraints.isNotEmpty()) allConstraints.addAll(constraints)
        var config: TrajectoryConfig =  TrajectoryConfig(maxVelocity.value, maxAcceleration.value)
        config.setEndVelocity(endVelocity.value)
        config.setReversed(reversed)
        config.addConstraints(allConstraints)
        return TrajectoryGenerator.generateTrajectory(
                points.map { it.position },
                config
        )
    }
}

fun Pose2d.asWaypoint() = TrajectoryWaypoints.Waypoint(this)
