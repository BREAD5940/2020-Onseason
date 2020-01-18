package frc.robot.subsystems

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import frc.robot.Constants
import org.junit.Test

class DriveTest {

    @Test
    fun testAngles() {

        val kinematics = Constants.kinematics
        val out = kinematics.toSwerveModuleStates(ChassisSpeeds(0.0, 0.0, 3.14))
        println(out.map { "${it.angle.degrees}" })

    }

}