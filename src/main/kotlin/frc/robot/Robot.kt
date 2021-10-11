
package frc.robot

import edu.wpi.first.math.WPIMathJNI
import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
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
import frc.robot.subsystems.shooter.ZeroCommand
import org.ghrobotics.lib.wrappers.FalconTimedRobot

object Robot : FalconTimedRobot() {

    val isEnabled get() = wrappedValue.isEnabled

    const val debugMode = false

    val led = AddressableLED(7)
    val buffer = AddressableLEDBuffer(100) // irl 43

    override fun robotInit() {
        Network // at the top because s3ndable choosers need to be instantiated
        Autonomous

        +DriveSubsystem
        +FlywheelSubsystem
        +IntakeSubsystem
        +HoodSubsystem
        +BumperGrabberSubsystem
        +VisionSubsystem

        SmartDashboard.putData(CommandScheduler.getInstance())

        super.robotInit()

        for (i in 0 until buffer.length) {

            buffer.setRGB(i, 0, 0, 0)
        }
        led.setLength(buffer.length)
        led.setData(buffer)
        led.start()
    }

    var last = 0

    override fun teleopPeriodic() {
    }

    override fun robotPeriodic() {
        Autonomous.update()
        Controls.update()
        Network.update()
//        println(FlywheelSubsystem.shooterMaster.encoder.position.inDegrees())

        for (i in 0 until buffer.length) {
            buffer.setRGB(i, 0, 0, 0)
        }
        buffer.setRGB(last, 100, 0, 0)
        led.setData(buffer)
        last++
        if (last >= buffer.length) last = 0
    }

    override fun disabledInit() {
    }

    override fun teleopInit() {
        ZeroCommand().schedule()
    }

    override fun autonomousInit() {
        HoodSubsystem.enabledReset()

        ShooterCharacterizationCommand().schedule()
    }
}

fun main() {

    edu.wpi.first.math.WPIMathJNI.forceLoad()

    WPIMathJNI.forceLoad()

    VecBuilder.fill(0.0)
    println("Starting on version ${WPILibVersion.Version}")
    Robot.start()
}
