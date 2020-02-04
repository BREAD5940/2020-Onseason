package frc.robot

import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.inMeters
import org.ghrobotics.lib.mathematics.units.inches

object Constants {

    val kBumperThickness = 3.25.inches
    val baseWidth = 29.inches // TODO check
    val baseLen = 17.5.inches // TODO check

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
}
