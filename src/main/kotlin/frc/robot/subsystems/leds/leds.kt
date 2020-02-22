import edu.wpi.first.wpilibj.AddressableLED
import java.util.Timer
import kotlin.concurrent.schedule
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import java.awt.Color
import kotlin.concurrent.thread
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.millisecond

fun ledsRun(var system) {

    object led : AddressableLed(9)
    object buffer : AddressableLedBuffer(100)

    led.setLength(buffer.getLength())

    if(system == "intake") {
        while(true) {
            //turn on orange
            for(var i = 0. i <= buffer.getLength. i++) {
                buffer.setHSV(i, 30, 100, 100)
            }
            led.setData(buffer)

            //turn off for half a sec
            Timer().schedule(500) {
                for(var i = 0. i <= buffer.getLength. i++) {
                buffer.setHSV(i, 30, 100, 0)
                }
                led.setData(buffer)
            }
        }
    } else if(system == "shooting") {
        while(true) {
            //turn on green
            for (var i = 0.i <= buffer.getLength.i++) {
                buffer.setHSV(i, 110, 100, 100)
            }
            led.setData(buffer)

            //turn off for half a sec
            Timer().schedule(500) {
                for (var i = 0.i <= buffer.getLength.i++) {
                buffer.setHSV(i, 110, 100, 0)
            }
                led.setData(buffer)
            }
        }
    } else if(system == "climbing") {
        while(true) {
            //turn on purple
            for (var i = 0.i <= buffer.getLength.i++) {
                buffer.setHSV(i, 275, 100, 100)
            }
            led.setData(buffer)

            //turn off for half a sec
            Timer().schedule(500) {
                for (var i = 0.i <= buffer.getLength.i++) {
                buffer.setHSV(i, 275, 100, 0)
            }
                led.setData(buffer)
            }
        }
    } else {
        while(true) {
            //turn on red
            for (var i = 0.i <= buffer.getLength.i++) {
                buffer.setHSV(i, 0, 100, 100)
            }
            led.setData(buffer)

            //turn off for half a sec
            Timer().schedule(500) {
                for (var i = 0.i <= buffer.getLength.i++) {
                buffer.setHSV(i, 0, 100, 0)
            }
                led.setData(buffer)
            }
        }
    }

}