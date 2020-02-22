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
    val pitchLookupTable3v3 = InterpolatingTable(
            // maybe we'll do target pitch for now?
            -3.9 to ShotParameter(67.degrees, 4000.revolutionsPerMinute),
            0.3 to ShotParameter(64.5.degrees, 3500.revolutionsPerMinute, (1).degrees),
            4.3 to ShotParameter(65.degrees, 2600.revolutionsPerMinute, (1).degrees),
            5.4 to ShotParameter(63.8.degrees, 2400.revolutionsPerMinute, (0.5).degrees),
            8.6 to ShotParameter(62.5.degrees, 2400.revolutionsPerMinute, (0.5).degrees),
            12.2 to ShotParameter(61.5.degrees, 2100.revolutionsPerMinute, 0.5.degrees),
            16.2 to ShotParameter(60.5.degrees, 1900.revolutionsPerMinute, 0.5.degrees)
    )

    /**
     * The default shot lookup table, in degrees of elevation to ShotParameters
     */
    val pitchLookupTable5v = InterpolatingTable(
            // maybe we'll do target pitch for now?
            -16.3 to ShotParameter(54.degrees, 5000.revolutionsPerMinute, -1.degrees),
            -16.3 to ShotParameter(54.degrees, 5000.revolutionsPerMinute)
    )

    /**
     * Lookup table to correlate distance in meters to shot parameters
     */
    val distanceLookupTable5v = InterpolatingTable(
            4.0 to ShotParameter(55.degrees, 4000.revolutionsPerMinute),
            5.0 to ShotParameter(51.5.degrees, 3500.revolutionsPerMinute, (0).degrees),
            6.0 to ShotParameter(52.degrees, 2600.revolutionsPerMinute, (0).degrees),
            7.0 to ShotParameter(50.3.degrees, 2400.revolutionsPerMinute, (0).degrees),
            8.0 to ShotParameter(48.3.degrees, 2400.revolutionsPerMinute, (0).degrees),
            9.0 to ShotParameter(46.8.degrees, 2100.revolutionsPerMinute, 0.5.degrees),
            9.9 to ShotParameter(45.3.degrees, 1900.revolutionsPerMinute, 0.5.degrees)
    )

    val rightBelowGoalParameter3v3 = ShotParameter(44.degrees, 1600.revolutionsPerMinute)
    val rightBelowGoalParameter5v = ShotParameter(21.degrees, 1600.revolutionsPerMinute)

}
