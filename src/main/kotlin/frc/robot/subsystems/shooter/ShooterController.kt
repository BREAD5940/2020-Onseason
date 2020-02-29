package frc.robot.subsystems.shooter

import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator
import edu.wpi.first.wpilibj.estimator.KalmanFilter
import edu.wpi.first.wpilibj.system.LinearSystem
import edu.wpi.first.wpilibj.system.LinearSystemLoop
import frc.robot.subsystems.drive.SwerveDriveOutput
import frc.team4069.keigen.*
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.*

object ShooterController {

    val plant = LinearSystem.identifyVelocitySystem(0.0194, 0.0004, 12.0)
    val filter = KalmanFilter(`1`, `1`, `1`, plant,
            vec(`1`).fill(1.0),
            vec(`1`).fill(0.02),
            0.020)

    val controller = LinearQuadraticRegulator(`1`, `1`, plant,
            vec(`1`).fill(6.0),
            vec(`1`).fill(12.0),
            0.020)

    val loop = LinearSystemLoop(`1`, `1`, `1`, plant, controller, filter)

    fun enable() = loop.enable()
    fun reset() = loop.reset()
    fun disable() = loop.disable()

    fun setSpeed(speed: SIUnit<Velocity<Radian>>) {
        loop.nextR = vec(`1`).fill(speed.value)
    }

    fun update(measuredSpeed: SIUnit<Velocity<Radian>>) {
        if(!loop.isEnabled) return

        loop.predict(0.020)
        loop.correct(vec(`1`).fill(measuredSpeed.value))
    }

    val nextU get() = loop.getU(0).volts
    val xHat get() = loop.getXHat(0).radians.velocity

}