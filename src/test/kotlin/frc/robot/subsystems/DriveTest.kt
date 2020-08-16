//package frc.robot.subsystems
//
//import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator
//import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
//import edu.wpi.first.wpilibj.system.LinearSystem
//import edu.wpi.first.wpilibj.system.plant.DCMotor
//import frc.robot.Constants
//import lib.createElevatorVelocitySystem
//import org.junit.Test
//import frc.team4069.keigen.*
//import org.ghrobotics.lib.mathematics.units.*
//
//class DriveTest {
//
//    @Test
//    fun testAngles() {
//
//        val plant = createElevatorVelocitySystem(DCMotor.getNEO(4), 150.kilo.grams.inLbs(),
//                2.inches.inMeters(), 8.31, 12.0)
//
//        val controller = LinearQuadraticRegulator(`1`, `1`, plant, vec(`1`).fill(6.inches.inMeters()), vec(`1`).fill(12.0), 1.0 / 1000.0)
//        controller.enable()
//        controller.reset()
//        controller.update(vec(`1`).fill(1.0), vec(`1`).fill(1.0))
//
//    }
//}
