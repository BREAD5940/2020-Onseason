
package frc.robot

import edu.wpi.first.math.WPIMathJNI
import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.system.plant.DCMotor
import edu.wpi.first.wpilibj.util.Units
import edu.wpi.first.wpilibj.util.WPILibVersion
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpiutil.math.VecBuilder
import frc.robot.autonomous.Autonomous
import frc.robot.subsystems.climb.BumperGrabberSubsystem
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.shooter.FlywheelSubsystem
import frc.robot.subsystems.shooter.HoodSubsystem
import frc.robot.subsystems.shooter.ShooterCharacterizationCommand
import frc.robot.subsystems.vision.VisionSubsystem
import org.ghrobotics.lib.wrappers.FalconTimedRobot
import kotlin.math.pow

object Robot : FalconTimedRobot() {

    val isEnabled get() = wrappedValue.isEnabled
    const val debugMode = false


    val sim = SingleJointedArmSim(
        DCMotor.getNEO(1), 10.0 * 60.0 / 24.0,
        0.425 * Units.inchesToMeters(6.0).pow(2) * 3.0 * 10.0, 0.0, // 0 because we dont care abt gravity
        Units.degreesToRadians(-225.0), Units.degreesToRadians(225.0),
        0.0, // also 0 coz we don't care about gravity
        false
    )

    val encoder = DutyCycleEncoder(0)
    val encoderSim = DutyCycleEncoderSim(encoder)
    var volts = 0.0

    override fun robotInit() {
        println(0.425 * Units.inchesToMeters(6.0).pow(2) * 3.0 * 10.0)
        SmartDashboard.putNumber("volts", 0.0)
        encoder.distancePerRotation = 360.0
    }

    override fun robotPeriodic() {
        volts = SmartDashboard.getNumber("volts", 0.0)
        SmartDashboard.putNumber("Encoder pos", encoder.distance)

        if (RobotBase.isSimulation()) {
            sim.setInputVoltage(volts)
            sim.update(0.020)
            encoderSim.setDistance(Units.radiansToDegrees(sim.angleRads))
        }
    }

//    val led = AddressableLED(7)
//    val buffer = AddressableLEDBuffer(100) // irl 43
//
//    override fun robotInit() {
//        Network // at the top because s3ndable choosers need to be instantiated
//        Autonomous
//
//        +DriveSubsystem
//        +FlywheelSubsystem
//        +IntakeSubsystem
//        +HoodSubsystem
//        +BumperGrabberSubsystem
//        +VisionSubsystem
//
//        SmartDashboard.putData(CommandScheduler.getInstance())
//
//        super.robotInit()
//
//        for (i in 0 until buffer.length) {
//
//            buffer.setRGB(i, 0, 0, 0)
//        }
//        led.setLength(buffer.length)
//        led.setData(buffer)
//        led.start()
//    }
//
//    var last = 0
//
//    override fun teleopPeriodic() {
//    }
//
//    override fun robotPeriodic() {
//        Autonomous.update()
//        Controls.update()
//        Network.update()
////        println(FlywheelSubsystem.shooterMaster.encoder.position.inDegrees())
//
//        for (i in 0 until buffer.length) {
//            buffer.setRGB(i, 0, 0, 0)
//        }
//        buffer.setRGB(last, 100, 0, 0)
//        led.setData(buffer)
//        last++
//        if (last >= buffer.length) last = 0
//    }
//
//    override fun disabledInit() {
//    }
//
//    override fun teleopInit() {
//        HoodSubsystem.enabledReset()
//    }
//
//    override fun autonomousInit() {
//        HoodSubsystem.enabledReset()
//
//        ShooterCharacterizationCommand().schedule()
//    }
}

fun main() {
    Robot.start()
}
