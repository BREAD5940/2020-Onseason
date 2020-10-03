package frc.robot.subsystems.shooter

import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator
import edu.wpi.first.wpilibj.estimator.KalmanFilter
import edu.wpi.first.wpilibj.system.LinearSystemLoop
import edu.wpi.first.wpilibj.system.plant.LinearSystemId
import edu.wpi.first.wpiutil.math.numbers.N1
import frc.team4069.keigen.*
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.*

object ShooterController {

    val dt = 0.020

    val plant = LinearSystemId.identifyVelocitySystem(0.023, 0.0005)
    val filter = KalmanFilter(`1`, `1`, plant,
            vec(`1`).fill(3.0),
            vec(`1`).fill(0.004),
            dt)

    val controller = LinearQuadraticRegulator<N1, N1, N1>(plant,
            vec(`1`).fill(0.05), // decrease to make more aggressive
            vec(`1`).fill(12.0),
            dt)

    val loop = LinearSystemLoop(plant, controller, filter, 12.0, 0.020)

    fun enable() {}
    fun reset() = loop.reset(vec(`1`).fill(0.0))
    fun disable() = {}

    fun setSpeed(speed: SIUnit<Velocity<Radian>>) {
        loop.nextR = vec(`1`).fill(speed.value)
    }

    fun update(measuredSpeed: SIUnit<Velocity<Radian>>) {
        loop.correct(vec(`1`).fill(measuredSpeed.value))
        loop.predict(dt)
    }

    val nextU get() = loop.getU(0).volts
    val xHat get() = loop.getXHat(0).radians.velocity
}
