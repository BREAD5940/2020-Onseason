
package frc.robot

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.autonomous.Autonomous
import frc.robot.subsystems.climb.BumperGrabberSubsystem
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.shooter.FlywheelSubsystem
import frc.robot.subsystems.shooter.HoodSubsystem
import frc.robot.subsystems.shooter.ShooterCharacterizationCommand
import frc.robot.subsystems.vision.VisionSubsystem
import org.ghrobotics.lib.mathematics.units.derived.inDegrees
import org.ghrobotics.lib.wrappers.FalconTimedRobot

object Robot : FalconTimedRobot() {

    val isEnabled get() = wrappedValue.isEnabled

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
    }

    override fun teleopPeriodic() {
    }

    override fun robotPeriodic() {
//        Autonomous.update()
        Controls.update()
        Network.update()
//        println(FlywheelSubsystem.shooterMaster.encoder.position.inDegrees())
    }

    override fun disabledInit() {
    }

    override fun teleopInit() {
        HoodSubsystem.enabledReset()
    }

    override fun autonomousInit() {
        HoodSubsystem.enabledReset()

        ShooterCharacterizationCommand().schedule()
    }
}

fun main() {
    Robot.start()
}
