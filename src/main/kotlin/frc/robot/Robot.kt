
package frc.robot

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.autonomous.Autonomous
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.sensors.Sensors
import org.ghrobotics.lib.wrappers.FalconTimedRobot

object Robot : FalconTimedRobot() {

    val isEnabled get() = wrappedValue.isEnabled

    override fun robotInit() {
        Network // at the top because s3ndable choosers need to be instantiated
        Autonomous

        // + for subsystems
        +DriveSubsystem

        SmartDashboard.putData(CommandScheduler.getInstance())

        super.robotInit()
    }

    override fun autonomousInit() {
    }

    override fun teleopPeriodic() {
    }

    override fun robotPeriodic() {
        Autonomous.update()
        Controls.update()
        Network.update()
        Sensors.update()
    }

    override fun disabledInit() {
    }

    override fun teleopInit() {
    }
}

fun main() {
    Robot.start()
}
