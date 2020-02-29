package frc.robot

import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics
import frc.robot.subsystems.shooter.ShotParameter
import lib.InterpolatingTable
import lib.revolutionsPerMinute
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.inMeters
import org.ghrobotics.lib.mathematics.units.inches

object Constants {

    val kBumperThickness = 3.25.inches

    // the center to center distance of the robot
    val baseWidth = 20.75.inches // TODO check
    val baseLen = 25.75.inches // TODO check

    val kIntakeToCenter = Pose2d(baseLen / 2, 0.feet, 0.degrees)

    /** Module speeds as (fl, fr, br, bl) */
    val kModulePositions = listOf(
            Translation2d(baseLen.inMeters() / 2.0, baseWidth.inMeters() / 2.0), // fl
            Translation2d(baseLen.inMeters() / 2.0, -baseWidth.inMeters() / 2.0), // fr
            Translation2d(-baseLen.inMeters() / 2.0, -baseWidth.inMeters() / 2.0), // br
            Translation2d(-baseLen.inMeters() / 2.0, baseWidth.inMeters() / 2.0) // bl
    )

    val kinematics = SwerveDriveKinematics(
            kModulePositions[0],
            kModulePositions[1],
            kModulePositions[2],
            kModulePositions[3]
    )

    /**
     * The default shot lookup table, in degrees of elevation to ShotParameters
     */
    val pitchLookupTable5v = InterpolatingTable(
            // maybe we'll do target pitch for now?
            -21.1 to ShotParameter(64.degrees, 4000.revolutionsPerMinute, -1.degrees),
            -18.5 to ShotParameter(64.degrees, 4000.revolutionsPerMinute, -0.degrees),
            -16.3 to ShotParameter(55.degrees, 3100.revolutionsPerMinute, -1.degrees),
            -11.8 to ShotParameter(55.degrees, 2500.revolutionsPerMinute, -3.degrees),
            -4.8 to ShotParameter(56.degrees, 2100.revolutionsPerMinute, -6.degrees),
            -4.5 to ShotParameter(56.degrees, 2100.revolutionsPerMinute, -5.degrees),
            17.0 to ShotParameter(42.5.degrees, 1800.revolutionsPerMinute, -6.degrees)
    )

    /**
     * Lookup table to correlate distance in meters to shot parameters
     */
    val distanceLookupTable5v = InterpolatingTable(
            3.65 to ShotParameter(53.degrees, 2150.revolutionsPerMinute, (-2).degrees),
            4.3 to ShotParameter(55.degrees, 2300.revolutionsPerMinute, (-2).degrees),
            5.15 to ShotParameter(58.degrees, 2400.revolutionsPerMinute, (-2).degrees),
            6.15 to ShotParameter(59.degrees, 2600.revolutionsPerMinute, (-2).degrees),
            6.7 to ShotParameter(59.degrees, 2800.revolutionsPerMinute, (-2).degrees),
            7.7 to ShotParameter(60.degrees, 2800.revolutionsPerMinute, (-2).degrees)
    )

    val rightBelowGoalParameter5v = ShotParameter(21.5.degrees, 2400.revolutionsPerMinute)

}
