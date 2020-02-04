package frc.robot.subsystems

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.system.LinearSystem
import edu.wpi.first.wpilibj.system.plant.DCMotor
import frc.robot.Constants
import org.junit.Test

class DriveTest {

    @Test
    fun testAngles() {

        val kinematics = Constants.kinematics
        val out = kinematics.toSwerveModuleStates(ChassisSpeeds(0.0, 0.0, 3.14))
        println(out.map { "${it.angle.degrees}" })

        val controller = LinearSystem.createElevatorSystem(DCMotor.getNEO(2), 3.0,
                .1, 0.81, 12.0)
    }
}
