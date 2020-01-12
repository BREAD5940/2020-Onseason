package frc.robot.autonomous.paths

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import frc.robot.Constants
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d

object TrajectoryWaypoints {

    /** Measured Field Coordinates **/

    // Habitat Zone

    /** Robot Starting Locations **/

    // Determine the starting X value for the robot.
    private val kStartX = kHabitatL2RX + Constants.kBumperThickness + Constants.baseLen / 2.0 - kRampHypotenuse

    // Starting on Level 1 HAB on the right side.
    val kSideStart = Pose2d(
            kHabitatL2RX + Constants.kBumperThickness + Constants.baseLen / 2.0 - kRampHypotenuse,
            kHabitatL2BY + Constants.kBumperThickness + Constants.baseWidth / 2.0,
            0.degrees
    )

    val kSideStartReversed = Pose2d(kSideStart.translation, 180.degrees)

    // Starting on Level 1 HAB in the center.
    val kCenterStart = Pose2d(kStartX, 13.5.feet)

    data class Waypoint(
        val trueLocation: Pose2d,
        val transform: Pose2d = Pose2d(),
        val translationalOffset: Translation2d = Translation2d(),
        val rotationalOffset: Rotation2d = Rotation2d(0.0)
    ) {

        val trueAndTransform = trueLocation + transform

        fun transformBy(other: Pose2d) = Waypoint(trueLocation, transform.transformBy(other), translationalOffset, rotationalOffset)

        val position = Pose2d(
                trueAndTransform.translation + translationalOffset,
                trueAndTransform.rotation + rotationalOffset
        )
    }
}

fun Pose2d(translation: Translation2d, rotation: SIUnit<Radian>) = Pose2d(translation, rotation.toRotation2d())
fun Pose2d.transformBy(other: Pose2d) = this.transformBy(edu.wpi.first.wpilibj.geometry.Transform2d(
        other.translation, other.rotation
))

operator fun Pose2d.plus(other: Pose2d): Pose2d = this.transformBy(other)!!

fun Pose2d.asWaypoint() = TrajectoryWaypoints.Waypoint(this)
