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
import org.ghrobotics.lib.mathematics.twodim.trajectory.FalconTrajectoryConfig
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.VelocityLimitRadiusConstraint
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.VelocityLimitRegionConstraint
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.* // ktlint-disable no-wildcard-imports

object TrajectoryFactory {

    /** Constraints **/

    val kMaxVelocity = 8.feet.velocity
    val kMaxAcceleration = 7.feet.acceleration

    /** Adjusted Poses **/

    // put Infinite Recharge poses here

    /** Trajectories **/

// big fancy trajectory
    val testTrajectory2 = generateTrajectory(
            false,
            listOf(
                    Pose2d(10.5.feet, 20.5.feet, 0.degrees),
                    Pose2d(18.5.feet, 24.5.feet, 0.degrees)
            ),
            getConstraints(kMaxVelocity), 6.feet.velocity, 8.feet.acceleration
    )

    fun Pose2d.withRotation(rotation: SIUnit<Radian>) = Pose2d(this.translation, rotation.toRotation2d())
    fun TrajectoryWaypoints.Waypoint.withRotation(rotation: SIUnit<Radian>) = Pose2d(this.position.translation, rotation.toRotation2d())

    val moveForward5Feet by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(9.371.feet, 25.709.feet, 0.degrees),
                        Pose2d(14.371.feet, 25.709.feet, 0.degrees)
                ),
                getConstraints(kMaxVelocity), kMaxVelocity, kMaxAcceleration
        )
    }
    val shootThree by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(11.75.feet, 25.689.feet, (-33).degrees),
                        Pose2d(11.75.feet, 19.262.feet, (-32).degrees)
                ),
                getConstraints(kMaxVelocity), kMaxVelocity, kMaxAcceleration
        )
    }

    val tenPCAutoToShieldGenerator by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(11.75.feet, 25.689.feet, 0.0.degrees),
                        Pose2d(20.383.feet, 18.592.feet, (-68).degrees)
                ),
                listOf(
                        SwerveDriveKinematicsConstraint(kinematics, kMaxVelocity.value),
                        VelocityLimitRegionConstraint(Rectangle2d(
                                Translation2d(19.723, 17.536),
                                Translation2d(23.344, 16.434)),
                                3.feet.velocity),
                        VelocityLimitRadiusConstraint(Translation2d(18.623, 16.526),
                                1.5.feet,
                                3.feet.velocity)
                ), kMaxVelocity, kMaxAcceleration
        )
    }

    val tenPCAutoShieldGeneratorToShoot by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(20.383.feet, 18.592.feet, (-68).degrees),
                        Pose2d(16.591.feet, 19.262.feet, 180.degrees)
                ),
                getConstraints(kMaxVelocity), kMaxVelocity, kMaxAcceleration
        )
    }

    val getPCFromTrench by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(11.75.feet, 19.262.feet, 48.degrees),
                        Pose2d(20.72.feet, 23.709.feet, 10.degrees),
                        Pose2d(35.223.feet, 24.995.feet, 0.degrees)
                ),
                // TODO check if encompasses both pc areas in trench

                listOf(
                        SwerveDriveKinematicsConstraint(kinematics, kMaxVelocity.value),
                        VelocityLimitRegionConstraint(Rectangle2d(
                                Translation2d(32.808, 22.449),
                                Translation2d(28.79, 26.972)),
                                3.feet.velocity)
//                        VelocityLimitRegionConstraint(Rectangle2d(
//                                Translation2d(20.075, 25.434),
//                                Translation2d(27.752, 23.918)),
//                                6.feet.velocity)
                ), kMaxVelocity, kMaxAcceleration
        )
    }

    val trenchToShoot by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(35.223.feet, 24.995.feet, 180.degrees),
                        Pose2d(24.185.feet, 24.168.feet, 180.degrees),
                        Pose2d(16.319.feet, 22.536.feet, 180.degrees)
                ),
                listOf(
                        SwerveDriveKinematicsConstraint(kinematics, kMaxVelocity.value),
                        VelocityLimitRegionConstraint(Rectangle2d(
                                Translation2d(32.808,22.449),
                                Translation2d(28.79,26.972)),
                                3.feet.velocity)
                        ), kMaxVelocity, kMaxAcceleration
                )
    }

    val sixPCGrab3FromTrench by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(9.5.feet, 22.feet, 22.degrees),
                        Pose2d(18.5.feet, 24.25.feet, 0.degrees),
                        Pose2d(34.feet, 24.25.feet, 0.degrees)
                ),
                getConstraints(kMaxVelocity) + VelocityLimitRadiusConstraint(
                        Pose2d(9.5.feet, 22.feet, 22.degrees).translation,
                        5.feet,
                        4.feet.velocity
                ), 6.feet.velocity, kMaxAcceleration
        )
    }

    val sixPCReturnFromTrench by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(34.feet, 24.25.feet, 180.degrees),
                        Pose2d(28.feet, 24.feet, 180.degrees),
                        Pose2d(9.feet, 20.feet, (-170).degrees)
                ),
                getConstraints(kMaxVelocity), 10.feet.velocity, kMaxAcceleration
        )
    }

    val tenPCAutoStartToOpposingTrench by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(11.79.feet, 1.281.feet, 0.degrees),
                        Pose2d(20.75.feet, 2.2.feet, 0.degrees),
                        Pose2d(19.607.feet, 2.2.feet, 0.degrees)
                ),
                listOf(
                        SwerveDriveKinematicsConstraint(kinematics, kMaxVelocity.value),
                        VelocityLimitRegionConstraint(Rectangle2d(
                                Translation2d(21.552, 4.357),
                                Translation2d(24.824, -0.051)),
                                3.feet.velocity)
                ), kMaxVelocity, kMaxAcceleration
        )
    }

    val tenPCAutoOpposingTrenchToShoot by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(19.607.feet, 2.2.feet, 180.degrees),
                        Pose2d(15.feet, 17.feet, 180.degrees)
                ),
                getConstraints(kMaxVelocity), kMaxVelocity, kMaxAcceleration
        )
    }

    val retrieve5FromShieldGenerator by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(15.feet, 17.feet, 67.degrees),
                        Pose2d(19.142.feet, 12.301.feet, 67.degrees),
                        Pose2d(18.575.feet, 13.746.feet, 67.degrees),
                        Pose2d(16.745.feet, 17.783.feet, 0.degrees),
                        Pose2d(20.514.feet, 18.4.feet, (-68).degrees)
                ),
                listOf(
                        SwerveDriveKinematicsConstraint(kinematics, kMaxVelocity.value),
                            VelocityLimitRadiusConstraint(Translation2d(18.624, 16.571),
                                    1.5.feet,
                                    4.feet.velocity),
                            VelocityLimitRegionConstraint(Rectangle2d(
                                    Translation2d(21.468, 11.5),
                                    Translation2d(19.02, 15.286)),
                                    4.feet.velocity), //subject to change
                            VelocityLimitRegionConstraint(Rectangle2d(
                                    Translation2d(22.479,17.24),
                                    Translation2d(18.976,17.215)),
                                    4.feet.velocity) //subject to change
                            ), kMaxVelocity, kMaxAcceleration
                    )

    }
    val tenPCShieldGeneratorToShoot by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(20.514.feet, 18.4.feet, (-68).degrees),
                        Pose2d(15.feet, 18.929.feet, 180.degrees)
                ),
                listOf(
                        SwerveDriveKinematicsConstraint(kinematics, kMaxVelocity.value),
                        VelocityLimitRadiusConstraint(Translation2d(18.624, 16.571),
                                1.5.feet,
                                3.feet.velocity)
                ), kMaxVelocity, kMaxAcceleration
        )
    }

    val sixPCStartToShoot by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(11.75.feet, 25.689.feet, 180.degrees),
                        Pose2d(16.319.feet, 22.536.feet, 180.degrees)
                ),
                getConstraints(kMaxVelocity), kMaxVelocity, kMaxAcceleration
        )
    }

    val shootFromTrench by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(30.826.feet, 24.719.feet, 0.0.degrees),
                        Pose2d(16.319.feet, 22.536.feet, 0.0.degrees)
                ),
                getConstraints(kMaxVelocity), kMaxVelocity, kMaxAcceleration
        )
    }

    val sixPCGetPCFromShieldGenerator by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(16.557.feet, 14.551.feet, 160.degrees),
                        Pose2d(18.65.feet, 13.541.feet, 24.degrees),
                        Pose2d(19.134.feet, 12.347.feet, 24.degrees)
                ),
                listOf(
                        SwerveDriveKinematicsConstraint(kinematics, kMaxVelocity.value),
                        VelocityLimitRegionConstraint(Rectangle2d(
                                Translation2d(21.468, 11.5),
                                Translation2d(19.02, 15.286)),
                                3.feet.velocity)
                ), kMaxVelocity, kMaxAcceleration
        )
    }

    val sixPCShieldGeneratorToShoot by lazy {
        generateTrajectory(
                false,
                listOf(
                        Pose2d(19.134.feet, 12.347.feet, 24.degrees),
                        Pose2d(16.557.feet, 14.551.feet, 160.degrees)
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
        points: List<Pose2d>,
        constraints: List<TrajectoryConstraint>,
        maxVelocity: SIUnit<Velocity<Meter>>,
        maxAcceleration: SIUnit<Acceleration<Meter>>,
        endVelocity: SIUnit<Velocity<Meter>> = 0.inches.velocity,
        clampedCubic: Boolean = true
    ): Trajectory {

        val allConstraints = ArrayList<TrajectoryConstraint>()

        if (constraints.isNotEmpty()) allConstraints.addAll(constraints)
        val config: TrajectoryConfig = TrajectoryConfig(maxVelocity.value, maxAcceleration.value)

        config.endVelocity = endVelocity.value
        config.isReversed = reversed
        config.addConstraints(allConstraints)
        return if (clampedCubic) TrajectoryGenerator.generateTrajectory(points.first(), points.subList(1, points.size - 1).map { it.translation }, points.last(), config)
        else TrajectoryGenerator.generateTrajectory(
                points,
                config)
    }
}

fun Pose2d.asWaypoint() = TrajectoryWaypoints.Waypoint(this)
