package frc.robot.subsystems.leds

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.ghrobotics.lib.commands.FalconSubsystem

object newLEDs : FalconSubsystem() {

    val LED = AddressableLED(7)
    val buffer = AddressableLEDBuffer(111) //TODO change LED length when more are added

    init {
        LED.setLength(buffer.length)

        LED.setData(buffer)
        LED.start()
    }

    private fun showBallAnimation(): Job = GlobalScope.launch {

        for (i in 0 until 19) {
            buffer.setRGB(i, 130, 24, 30)
        }

        delay(10)

        for (i in 92 until 112) {
            buffer.setRGB(i, 130, 24, 30)
        }

        delay(10)

        for (i in 75 until 92) {
            buffer.setRGB(i, 130, 24, 30)
        }

        delay(10)

        for (i in 65 until 75) {
            buffer.setRGB(i, 130, 24, 30)
        }

        delay(10)

        for (i in 19 until 44) {
            buffer.setRGB(i, 130, 24, 30)
        }

        delay(10)

        for (i in 62 until 65) {
            buffer.setRGB(i, 130, 24, 30)
        }

        delay(10)

        for (i in 44 until 62) {
            buffer.setRGB(i, 130, 24, 30)
        }

        LED.setData(buffer)
        LED.start()
    }

    private fun  rainbow() : Job = GlobalScope.launch {
        var firstPixelHue = 0
        for (i in 0 until buffer.length) {
            var hue = (firstPixelHue + (i * 180 / buffer.length)) % 180
            buffer.setHSV(i, hue, 255, 255)
        }
        firstPixelHue += 3
        firstPixelHue %= 180
    }

    fun intakeMode(shooterSensor: Boolean) { //TODO: replace w/ real output, will be analog output
        if (shooterSensor) {
            showBallAnimation()
        } else {
            rainbow()
        }
        LED.setData(buffer)
        LED.start()
    }
}