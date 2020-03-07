package frc.robot.subsystems

import frc.robot.subsystems.superstructure.TheLEDs
import org.junit.Test

class LedTest {

    @Test fun testLed() {

        val led = TheLEDs
        led.lateInit()
        Thread.sleep(10000)

    }

}