package frc.robot.auto.paths

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.trajectory.Trajectory
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint
import frc.robot.Constants.kinematics
import frc.robot.autonomous.paths.TrajectoryWaypoints
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.VelocityLimitRadiusConstraint
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.VelocityLimitRegionConstraint
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.* // ktlint-disable no-wildcard-imports

object TrajectoryFactory {

    /** Constraints **/

    val kMaxVelocity = 4.feet.velocity
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
                        Pose2d(11.75.feet, 25.689.feet, 270.degrees).asWaypoint(),
                        Pose2d(11.5.feet, 22.feet, (270).degrees).asWaypoint()
                ),
                getConstraints(kMaxVelocity), kMaxVelocity, kMaxAcceleration
        )
    }

    val tenPCAutoToShieldGenerator by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(11.75.feet, 25.689.feet, 0.0.degrees).asWaypoint(),
                        Pose2d(20.383.feet, 18.592.feet, (-68).degrees).asWaypoint()
                ),
                getConstraints(kMaxVelocity), kMaxVelocity, kMaxAcceleration
        )
    }

    val tenPCAutoShieldGeneratorToShoot by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(20.383.feet, 18.592.feet, (-68).degrees).asWaypoint(),
                        Pose2d(17.682.feet, 21.939.feet, (-111).degrees).asWaypoint()
                ),
                getConstraints(kMaxVelocity), kMaxVelocity, kMaxAcceleration
        )
    }

    val getPCFromTrench by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(11.5.feet, 22.feet, (270).degrees).asWaypoint()
                        Pose2d(17.682.feet, 21.939.feet, (-111).degrees).asWaypoint(),
                        Pose2d(19.445.feet, 24.719.feet, 0.0.degrees).asWaypoint(),
                        Pose2d(30.826.feet, 24.719.feet, 0.0.degrees).asWaypoint()
                ),
                listOf(
                        SwerveDriveKinematicsConstraint(kinematics, kMaxVelocity.value),
                        VelocityLimitRegionConstraint(Rectangle2d(
                                Translation2d(32.574, 22.449),
                                Translation2d(29.09, 27.041)),
                                3.feet.velocity)
                ), kMaxVelocity, kMaxAcceleration
        )
    }

    val trenchToShoot by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(30.826.feet, 24.719.feet, 180.0.degrees).asWaypoint(),
                        Pose2d(27.879.feet, 24.719.feet, 180.0.degrees).asWaypoint(),
                        Pose2d(17.682.feet, 21.939.feet, (-111).degrees).asWaypoint()
                ),
                getConstraints(kMaxVelocity), kMaxVelocity, kMaxAcceleration
        )
    }

    val eightPCAutoStartToOpposingTrench by lazy {
        generateTrajectory(
                false,
                listOf(
                    Pose2d(11.79.feet, 1.281.feet, 0.0.degrees).asWaypoint(),
                    Pose2d(20.119.feet, 2.337.feet, 0.0.degrees).asWaypoint()
                ),
                listOf(
                        SwerveDriveKinematicsConstraint(kinematics, kMaxVelocity.value),
                        VelocityLimitRegionConstraint(Rectangle2d(
                                Translation2d(21.552, 4.357),
                                Translation2d(24.824, -0.051)
                        ), 3.feet.velocity)
                ), kMaxVelocity, kMaxAcceleration
        )
    }

    val eightPCAutoOpposingTrenchToShoot by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(20.119.feet, 2.337.feet, 0.0.degrees).asWaypoint(),
                        Pose2d(16.557.feet, 14.551.feet, 160.degrees).asWaypoint()
                ),
                listOf(
                        SwerveDriveKinematicsConstraint(kinematics, kMaxVelocity.value),
                        VelocityLimitRadiusConstraint(Translation2d(18.624, 16.571), 1.5.feet, 3.feet.velocity)
                ), kMaxVelocity, kMaxAcceleration
        )
    }

    val eightPCAutoShootToShieldGenerator by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(16.557.feet, 14.551.feet, 160.degrees).asWaypoint(),
                        Pose2d(18.65.feet, 13.541.feet, 24.degrees).asWaypoint(),
                        Pose2d(19.134.feet, 12.347.feet, 24.degrees).asWaypoint()
                ),
                getConstraints(kMaxVelocity), kMaxVelocity, kMaxAcceleration
        )
    }

    val eightPCShieldGeneratorToShoot by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(11.79.feet, 1.281.feet, 0.0.degrees).asWaypoint(),
                        Pose2d(16.557.feet, 14.551.feet, 160.degrees).asWaypoint()
                ),
                getConstraints(kMaxVelocity), kMaxVelocity, kMaxAcceleration
        )
    }

    val sixPCStartToShoot by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(12.1.feet, 25.5.feet, 270.degrees).asWaypoint(),
                        Pose2d(12.1.feet, 22.5.feet, 270.degrees).asWaypoint()
                ),
                getConstraints(kMaxVelocity), kMaxVelocity, kMaxAcceleration
        )
    }

    val sixPCGetPCFromShieldGenerator by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(16.557.feet, 14.551.feet, 160.degrees).asWaypoint(),
                        Pose2d(18.65.feet, 13.541.feet, 24.degrees).asWaypoint(),
                        Pose2d(19.134.feet, 12.347.feet, 24.degrees).asWaypoint()
                ),
                getConstraints(kMaxVelocity), 4.feet.velocity, kMaxAcceleration
        )
    }

    val sixPCShieldGeneratorToShoot by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(19.134.feet, 12.347.feet, 24.degrees).asWaypoint(),
                        Pose2d(16.557.feet, 14.551.feet, 160.degrees).asWaypoint()
                ),
                getConstraints(kMaxVelocity), kMaxVelocity, kMaxAcceleration
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
