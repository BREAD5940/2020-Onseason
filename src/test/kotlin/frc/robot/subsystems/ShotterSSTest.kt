package frc.robot.subsystems

import frc.robot.subsystems.shooter.ShooterController
import org.junit.Test

class ShotterSSTest {

    @Test fun test() {
        val k = ShooterController.controller.k[0, 0]
        println(k)
    }

}