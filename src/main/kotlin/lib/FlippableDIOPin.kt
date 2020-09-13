package lib

import edu.wpi.first.hal.DIOJNI
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.DigitalSource
import edu.wpi.first.wpilibj.Sendable
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry

/**
 * A port that can be an input or an output. Defaults to input.
 */
class FlippableDIOPin(private val port: Int) : DigitalSource(), Sendable, AutoCloseable {

    private var isInput = true
    private var handle = DIOJNI.initializeDIOPort(HAL.getPort(channel.toByte()), isInput)

    /**
     * Set the value of a digital output.
     *
     * @param value true is on, off is false
     */
    fun set(value: Boolean) {
        if (value) {
            // if we want output check if we need to flip the port from out to in
            if (isInput) {
                // we need to re register as an output
                DIOJNI.freeDIOPort(handle)
                DIOJNI.initializeDIOPort(HAL.getPort(channel.toByte()), false)
                isInput = false
            }
            DIOJNI.setDIO(handle, (1).toShort())
        } else { // Otherwise we want to disable the port -- check if we need to flip to input mode
            if (!isInput) {
                // we need to re register as an input
                DIOJNI.setDIO(handle, (0).toShort())
                DIOJNI.freeDIOPort(handle)
                DIOJNI.initializeDIOPort(HAL.getPort(channel.toByte()), true)
                isInput = true
            }
        }
    }

    /**
     * Gets the value being output from the Digital Output.
     *
     * @return the state of the digital output.
     */
    fun get(): Boolean {
        return DIOJNI.getDIO(handle)
    }

    override fun getAnalogTriggerTypeForRouting() = 0

    override fun getChannel() = port

    override fun getPortHandleForRouting() = handle

    override fun isAnalogTrigger() = false

    override fun close() {
        super.close()
        SendableRegistry.remove(this)

        DIOJNI.freeDIOPort(handle)
        handle = 0
    }

    override fun initSendable(builder: SendableBuilder?) {
        TODO("not implemented") // To change body of created functions use File | Settings | File Templates.
    }
}
