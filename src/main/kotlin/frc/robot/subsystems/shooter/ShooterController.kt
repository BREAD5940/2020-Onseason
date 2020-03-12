package frc.robot.subsystems.shooter

import edu.wpi.first.wpilibj.controller.FishyLinearQuadraticRegulator
import edu.wpi.first.wpilibj.controller.FishyLinearSystemLoop
import edu.wpi.first.wpilibj.estimator.KalmanFilter
import edu.wpi.first.wpilibj.system.LinearSystem
import frc.team4069.keigen.*
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.*

object ShooterController {

    val dt = 0.020

    val plant = LinearSystem.identifyVelocitySystem(0.023, 0.0005, 12.0)
    val filter = KalmanFilter(`1`, `1`, `1`, plant,
            vec(`1`).fill(3.0),
            vec(`1`).fill(0.004),
            dt)

    val controller = FishyLinearQuadraticRegulator(`1`, `1`, plant,
            vec(`1`).fill(0.05), // decrease to make more aggressive
            vec(`1`).fill(12.0),
            dt).apply {
        this.m_K = vec(`1`).fill(0.1 * 0.8)
    }

    val loop = FishyLinearSystemLoop(`1`, `1`, `1`, plant, controller, filter)

    fun enable() = loop.enable()
    fun reset() = loop.reset()
    fun disable() = loop.disable()

    fun setSpeed(speed: SIUnit<Velocity<Radian>>) {
        loop.nextR = vec(`1`).fill(speed.value)
    }

    fun update(measuredSpeed: SIUnit<Velocity<Radian>>) {
        if(!loop.isEnabled) return

        loop.correct(vec(`1`).fill(measuredSpeed.value))
        loop.predict(dt)
    }

    val nextU get() = loop.getU(0).volts
    val xHat get() = loop.getXHat(0).radians.velocity

}