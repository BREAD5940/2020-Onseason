package frc.test

import frc.test.SwerveEvadeTest.Constants.kModulePositions
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.junit.Test

class SwerveStopTest {
//    fun something() {
//        var blob = ArrayList<Translation2d>()
//        for (i in kModulePositions) {
//            if (i.useState == SwerveDriveOutput.Nothing) {
//                blob.add(i)
//            }
//        }
//
//        if (blob.size == 4) {
//
//        }
//    }

    @Test
    fun testInwards() {
        val `in` = pointInwards()
        println(`in`)
    }

    fun pointInwards(): List<SwerveModuleState> {
        // what we'll do it just rotate, then rotate the angles by 90 degrees
        // and set the speeds to 0


        return SwerveEvadeTest.Constants.kinematics.toSwerveModuleStates(ChassisSpeeds(
                0.0, 0.0, 1.0))
                .map { it -> SwerveModuleState(0.0, it.angle + 90.degrees.toRotation2d()) }

    }

}