package frc.robot.subsystems

import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator
import edu.wpi.first.wpilibj.system.LinearSystem
import edu.wpi.first.wpiutil.math.MatBuilder
import edu.wpi.first.wpiutil.math.MatrixUtils
import frc.robot.subsystems.shooter.ShooterController
import frc.team4069.keigen.*
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.inRadians
import org.ghrobotics.lib.mathematics.units.inMeters
import org.ghrobotics.lib.mathematics.units.inches
import org.junit.Test
import kotlin.math.PI

class StateSpaceMemes {

    @Test
    fun testSwerveTrajectoryGains() {
        val model = LinearSystem(
                `3`, `3`, `3`,
                MatrixUtils.zeros(`3`, `3`),
                MatrixUtils.eye(`3`),
                MatrixUtils.eye(`3`),
                MatrixUtils.zeros(`3`, `3`),
                vec(`3`).fill(-3.5, -3.5, -3.0), // umin
                vec(`3`).fill(3.5, 3.5, 3.0) // umax
        )

        val controller = LinearQuadraticRegulator(
                `3`, `3`, model,
                vec(`3`).fill(5.inches.inMeters(), 5.inches.inMeters(), 5.degrees.inRadians()),
                vec(`3`).fill(1.0, 1.0, 2.0),
                0.020
        )

        println(controller.k.storage)

        val wheelPlant = LinearSystem.identifyVelocitySystem(2.9, 0.3, 12.0)
        val wheelController = LinearQuadraticRegulator(
                `1`, `1`, wheelPlant,
                vec(`1`).fill(3.inches.inMeters()),
                vec(`1`).fill(12.0),
                1.0 / 1000.0
        )

        val kp = wheelController.k[0] // volts per meter per sec of error
        // v = r w, or w = v / r
        // so kp_rad = (volts / meter per sec) * r meters / radians
        val kpRad = kp * 2.inches.inMeters()
        val kpRot = kpRad / 2.0 / PI
        val kpRotPerMinute = kpRot / 60.0
        val kpOutputRotPerMinte = kpRotPerMinute / 12.0

        println(kpOutputRotPerMinte)
    }

}