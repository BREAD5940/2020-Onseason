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

fun ledsRun() {

    object led : AddressableLed(9)
    object buffer : AddressableLedBuffer(100)

    led.setLength

}