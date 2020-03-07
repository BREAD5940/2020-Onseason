package frc.robot.subsystems.superstructure

import edu.wpi.first.wpilibj.AddressableLED
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import java.awt.Color
import kotlin.concurrent.thread
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.DriverStation
import org.ghrobotics.lib.commands.FalconSubsystem
import frc.robot.Controls
import org.ghrobotics.lib.mathematics.units.millisecond

object TheLEDs : FalconSubsystem() {
    object LEDs : AddressableLED(9) //
    object LEDBuffer : AddressableLEDBuffer(100)
    var allianceHue = 85 // GREEN (120/360 on color wheel)

    override fun lateInit() {
        LEDs.setLength(LEDBuffer.length)
        LEDs.setData(LEDBuffer)
        LEDs.start()

        if (DriverStation.getInstance().alliance == DriverStation.Alliance.Red) {
            allianceHue = 0      // RED (0/360 on color wheel)
        } else if (DriverStation.getInstance().alliance == DriverStation.Alliance.Blue) {
            allianceHue = 170    // BLUE (240/360 on color wheel)
        }

        updateThread.start()
    }

    private val updateThread = thread(start = false) {
        var firstPixelValue = 0
        var hue = allianceHue


        while (true) {
            // For every pixel
            for (i in 0 until LEDBuffer.length) {
                val value = (firstPixelValue + (i * 255 / LEDBuffer.length)) % 255
                // Set the value
                LEDBuffer.setHSV(i, hue, 255, value)
            }

            // Increase by to make the rainbow "move"
            firstPixelValue += 3
            // Check bounds
            firstPixelValue %= 255

            if (Controls.isClimbing) {
                hue = 277
            } else {
                hue = allianceHue
            }

            println("hello")

        }


        // Thread.sleep(wantedState.blinkTime.millisecond.toLong() / 2)
    }
}
