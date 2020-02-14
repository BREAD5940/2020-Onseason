package frc.robot.auto.paths

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.trajectory.Trajectory
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint
import frc.robot.Constants.kinematics
import frc.robot.autonomous.paths.TrajectoryWaypoints
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.* // ktlint-disable no-wildcard-imports

object TrajectoryFactory {

    /** Constraints **/

    val kMaxVelocity = 7.5.feet.velocity
    val kMaxAcceleration = 6.feet.acceleration

    /** Adjusted Poses **/

    // put Infinite Recharge poses here

    /** Trajectories **/

// big fancy trajectory
    val testTrajectory2 = generateTrajectory(
            false,
            listOf(
                    Pose2d(5.396.feet, 5.551.feet, 0.degrees).asWaypoint(),
                    Pose2d(10.004.feet, 5.648.feet, 0.degrees).asWaypoint(),
                    Pose2d(8.334.feet, 2.474.feet, (-145.637).degrees).asWaypoint(),
                    Pose2d(7.102.feet, 6.796.feet, 71.162.degrees).asWaypoint()
            ),
            getConstraints(kMaxVelocity), 5.feet.velocity, kMaxAcceleration
    )

    fun Pose2d.withRotation(rotation: SIUnit<Radian>) = Pose2d(this.translation, rotation.toRotation2d())
    fun TrajectoryWaypoints.Waypoint.withRotation(rotation: SIUnit<Radian>) = Pose2d(this.position.translation, rotation.toRotation2d()).asWaypoint()

    val moveForward5Feet by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(9.371.feet, 25.709.feet, 0.degrees).asWaypoint(),
                        Pose2d(14.371.feet, 25.709.feet, 0.degrees).asWaypoint()
                ),
                getConstraints(kMaxVelocity), kMaxVelocity, kMaxAcceleration
        )
    }
    val shootThree by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(9.371.feet, 25.709.feet, 0.degrees).asWaypoint(),
                        Pose2d(12.16.feet, 21.393.feet, 180.degrees).asWaypoint()
                ),
                getConstraints(kMaxVelocity), kMaxVelocity, kMaxAcceleration
        )
    }
    val grabTwoFromTrench by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(9.371.feet, 25.709.feet, 0.degrees).asWaypoint(),
                        Pose2d(20.779.feet, 24.699.feet, 0.degrees).asWaypoint(),
                        Pose2d(24.002.feet, 24.551.feet, 0.degrees).asWaypoint(),
                        Pose2d(12.16.feet, 21.393.feet, 180.degrees).asWaypoint()
                ),
                getConstraints(kMaxVelocity), kMaxVelocity, kMaxAcceleration
        )
    }
    val grabThreeFromTrench by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(12.16.feet, 21.393.feet, 180.degrees).asWaypoint(),
                        Pose2d(20.866.feet, 24.791.feet, 0.degrees).asWaypoint(),
                        Pose2d(23.958.feet, 24.597.feet, 0.degrees).asWaypoint(),
                        Pose2d(26.956.feet, 24.658.feet, 0.degrees).asWaypoint(),
                        Pose2d(12.16.feet, 21.393.feet, 0.degrees).asWaypoint(),
                        Pose2d(19.305.feet, 23.74.feet, 200.degrees).asWaypoint()
                ),
                getConstraints(kMaxVelocity), kMaxVelocity, kMaxAcceleration
        )
    }

    val grabFiveAndShootTrench by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(12.16.feet, 21.393.feet, 180.degrees).asWaypoint(),
                        Pose2d(20.69.feet, 24.612.feet, 0.degrees).asWaypoint(),
                        Pose2d(23.812.feet, 24.658.feet, 0.degrees).asWaypoint(),
                        Pose2d(27.11.feet, 24.704.feet, 0.degrees).asWaypoint(),
                        Pose2d(32.343.feet, 25.439.feet, 0.degrees).asWaypoint(),
                        Pose2d(32.409.feet, 23.878.feet, 90.degrees).asWaypoint(),
                        Pose2d(19.217.feet, 23.556.feet, 210.degrees).asWaypoint()
                        ),
                getConstraints(4.feet.velocity), 4.feet.velocity, 7.feet.acceleration
        )
    }
    val grabFiveAndShoot by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(12.16.feet, 21.393.feet, 180.degrees).asWaypoint(),
                        Pose2d(20.69.feet, 24.612.feet, 0.degrees).asWaypoint(),
                        Pose2d(23.812.feet, 24.658.feet, 0.degrees).asWaypoint(),
                        Pose2d(27.11.feet, 24.704.feet, 0.degrees).asWaypoint(),
                        Pose2d(32.343.feet, 25.439.feet, 0.degrees).asWaypoint(),
                        Pose2d(32.409.feet, 23.878.feet, 90.degrees).asWaypoint(),
                        Pose2d(11.983.feet, 18.781.feet, 180.degrees).asWaypoint()
                ),
                getConstraints(4.feet.velocity), 4.feet.velocity, 7.feet.acceleration
        )
    }
    val grabFiveAndShootShieldGenerator by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(12.16.feet, 21.393.feet, 180.degrees).asWaypoint(),
                        Pose2d(20.25.feet, 16.76.feet, 2.degrees).asWaypoint(),
                        Pose2d(21.613.feet, 17.311.feet, 180.degrees).asWaypoint(),
                        Pose2d(20.8.feet, 12.214.feet, 180.degrees).asWaypoint(),
                        Pose2d(20.448.feet, 13.408.feet, 0.degrees).asWaypoint(),
                        Pose2d(19.656.feet, 14.694.feet, 0.degrees).asWaypoint(),
                        Pose2d(12.16.feet, 21.393.feet, 180.degrees).asWaypoint()
                ),
                getConstraints(4.feet.velocity), 4.feet.velocity, 7.feet.acceleration
        )
    }
    val tenPointAutoMaybe by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(11.816.feet, 18.592.feet, -68.degrees).asWaypoint(),
                        Pose2d(20.383.feet, 18.592.feet, -68.degrees).asWaypoint(),
                        Pose2d(17.682.feet, 21.939.feet, -180.degrees).asWaypoint(),
                        Pose2d(19.445.feet, 24.719.feet, 0.0.degrees).asWaypoint(),
                        Pose2d(22.479.feet, 24.719.feet, 0.0.degrees).asWaypoint(),
                        Pose2d(25.513.feet, 24.719.feet, 0.0.degrees).asWaypoint(),
                        Pose2d(30.826.feet, 24.719.feet, 0.0.degrees).asWaypoint(),
                        Pose2d(17.682.feet, 21.939.feet, -180.degrees).asWaypoint()
                ),
                getConstraints(10.feet.velocity), 10.feet.velocity, 4.feet.acceleration
        )
    }

    /** Generation **/

    private fun getConstraints(
        maxSpeedMetersPerSecond: SIUnit<Velocity<Meter>>
    ) = listOf(SwerveDriveKinematicsConstraint(kinematics, maxSpeedMetersPerSecond.value))

    fun generateTrajectory(
        reversed: Boolean,
        points: List<TrajectoryWaypoints.Waypoint>,
        constraints: List<TrajectoryConstraint>,
        maxVelocity: SIUnit<Velocity<Meter>>,
        maxAcceleration: SIUnit<Acceleration<Meter>>,
        endVelocity: SIUnit<Velocity<Meter>> = 0.inches.velocity
    ): Trajectory {

        val allConstraints = ArrayList<TrajectoryConstraint>()

        if (constraints.isNotEmpty()) allConstraints.addAll(constraints)
        var config: TrajectoryConfig = TrajectoryConfig(maxVelocity.value, maxAcceleration.value)
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
