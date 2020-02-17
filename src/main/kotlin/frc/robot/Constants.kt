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
    val defaultShotLookupTable = InterpolatingTable(
            // maybe we'll do target pitch for now?
            -13.0 to ShotParameter(69.degrees, 4100.revolutionsPerMinute,0.degrees),
            -11.0 to ShotParameter(69.degrees, 3350.revolutionsPerMinute,0.degrees),
            -10.4 to ShotParameter(68.5.degrees, 3250.revolutionsPerMinute, 2.degrees),
            -6.2 to ShotParameter(68.5.degrees, 3200.revolutionsPerMinute, 0.degrees),
            -3.5 to ShotParameter(68.degrees, 3000.revolutionsPerMinute),
            0.1 to ShotParameter(65.5.degrees, 2800.revolutionsPerMinute, (1).degrees),
            4.4 to ShotParameter(65.degrees, 2300.revolutionsPerMinute, (1).degrees),
            5.1 to ShotParameter(64.4.degrees, 2200.revolutionsPerMinute, (0.5).degrees),
            8.6 to ShotParameter(63.degrees, 2200.revolutionsPerMinute, (0.5).degrees),
            12.2 to ShotParameter(62.degrees, 2100.revolutionsPerMinute, 0.5.degrees),
            16.2 to ShotParameter(59.degrees, 1850.revolutionsPerMinute, 0.5.degrees)
    )

    val rightBelowGoalParameter = ShotParameter(44.degrees, 1600.revolutionsPerMinute)
}
