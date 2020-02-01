package frc.test

import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics
import frc.test.SwerveEvadeTest.Constants.baseLen
import frc.test.SwerveEvadeTest.Constants.baseWidth
import frc.test.SwerveEvadeTest.Constants.kModulePositions
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.inMeters
import org.ghrobotics.lib.mathematics.units.inches
import org.junit.Test
import org.junit.Assert.*
import java.lang.Math.atan2
import java.util.ArrayList
import java.util.Scanner


class SwerveEvadeTest {

    object Constants {
        val kBumperThickness = 3.25.inches
        val baseWidth = 17.5.inches // TODO check
        val baseLen = 29.inches // TODO check

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

    @Test
    fun testForwardEvasion() {

        val positions = kModulePositions

        val driveVector = Translation2d(1.0, 0.0)

        val wheels = findEvasionWheels(driveVector, positions)

        // expected, actual
        val text = "hi! we chose ${positions.map { it.x }}"
        println(text)
        assertEquals(positions[0], wheels[0]) // ex. expected front left, we choose $wheels[0]


    }

    private fun findEvasionWheels(driveVector: Translation2d, positions: List<Translation2d>): List<Translation2d> {
        // put code here to...
        var modulesssss = ArrayList<Rotation2d>()
        // find angle of wheels (fl, fr, br, bl)
        for (i in kModulePositions) {
            var convertTranslationToRotation = Rotation2d(baseWidth.value, baseLen.value)
            modulesssss.add(convertTranslationToRotation)
        }
        // find angle of drive vector using Rotation2d(x, y)
        // figure out if angle is...
        //    between fl and fr,
        //    between fr and br,
        //    between fl and bl
        //    otherwise must be bl and br
        // return the Clockwise rotation center and CCW rotation center as a listOf(cw, ccw)
        var cw = Translation2d()
        var ccw = Translation2d()
        val driveVectorAngle = Rotation2d(driveVector.x, driveVector.y)
        if (driveVectorAngle.degrees <= modulesssss[0].degrees && driveVectorAngle.degrees > modulesssss[1].degrees) { //fl & fr
            cw = kModulePositions[0]
            ccw = kModulePositions[1]
            return listOf<Translation2d>(cw, ccw)
        } else if (driveVectorAngle.degrees <= modulesssss[1].degrees && driveVectorAngle.degrees > modulesssss[2].degrees) { //fr & br
            val cw = kModulePositions[1]
            val ccw = kModulePositions[2]
            return listOf<Translation2d>(cw, ccw)
        } else if (driveVectorAngle.degrees <= modulesssss[2].degrees && driveVectorAngle.degrees > modulesssss[3].degrees) { //br & bl
            val cw = kModulePositions[2]
            val ccw = kModulePositions[3]
            return listOf<Translation2d>(cw, ccw)
        } else { //bl & fl
            val cw = kModulePositions[3]
            val ccw = kModulePositions[0]
            return listOf<Translation2d>(cw, ccw)
        }
    }
}
