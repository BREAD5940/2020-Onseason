package frc.robot.subsystems.superstructure

import edu.wpi.first.wpilibj.AddressableLED
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import java.awt.Color
import kotlin.concurrent.thread
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.millisecond

class TheLEDs : FalconSubsystem() {
    object LEDs : AddressableLED(9) //
    object LEDBuffer : AddressableLEDBuffer(100)

    override fun lateInit() {
        LEDs.setLength(LEDBuffer.length)
        LEDs.setData(LEDBuffer)
        LEDs.start()

        updateThread.start()
    }

    private val updateThread = thread {
        var firstPixelValue = 0

        while (true) {
            // For every pixel
            for (i in 0..LEDBuffer.length) {
                val value = (firstPixelValue + (i * 255 / LEDBuffer.length)) % 255
                // Set the value
                LEDBuffer.setHSV(i, 0, 255, value)
            }

            // Increase by to make the rainbow "move"
            firstPixelValue += 3
            // Check bounds
            firstPixelValue %= 255
        }

        // Thread.sleep(wantedState.blinkTime.millisecond.toLong() / 2)
    }
}
