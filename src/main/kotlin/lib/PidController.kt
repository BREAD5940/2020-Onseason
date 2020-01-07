package lib

import kotlin.math.abs

@Deprecated("2910 PidController is depreciated", ReplaceWith("PIDController",
        "edu.wpi.first.wpilibj.controller.PIDController"))
class PidController(private val kP: Double, private val kD: Double) {

    private var setpoint: Double = 0.0

    private var continuous: Boolean = true
    private var inputRange = Double.POSITIVE_INFINITY
    private var minOutput = Double.NEGATIVE_INFINITY
    private var maxOutput = Double.POSITIVE_INFINITY

    var lastError = Double.NaN

    fun calculate(current: Double, dt: Double): Double {
        var error = setpoint - current
        if (continuous) {
            error %= inputRange
            if (abs(error) > inputRange / 2.0) {
                if (error > 0.0) {
                    error -= inputRange
                } else {
                    error += inputRange
                }
            }
        }

        var derivative = 0.0
        if (lastError.isFinite()) {
            derivative = (error - lastError) / dt
        }
        lastError = error

        return (kP * error + kD * derivative).coerceIn(minOutput, maxOutput)
    }

    fun reset() {
        lastError = Double.NaN
    }

    fun getSetpoint(): Double {
        return setpoint
    }

    fun setSetpoint(setpoint: Double) {
        this.setpoint = setpoint
    }

    fun setContinuous(continuous: Boolean) {
        this.continuous = continuous
    }

    fun setInputRange(minInput: Double, maxInput: Double) {
        this.inputRange = maxInput - minInput
    }

    /**
     * Sets the output range for the controller. Outputs will be clamped between these two values.
     *
     * @param min the minimum allowable output value
     * @param max the maximum allowable output value
     */
    fun setOutputRange(min: Double, max: Double) {
        if (max < min) {
            throw IllegalArgumentException("Minimum output cannot be greater than maximum output")
        }

        minOutput = min
        maxOutput = max
    }
}
