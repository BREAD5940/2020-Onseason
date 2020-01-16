
package frc.robot

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.autonomous.Autonomous
import frc.robot.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.ghrobotics.lib.wrappers.FalconTimedRobot

object Robot : FalconTimedRobot() {

    val isEnabled get() = wrappedValue.isEnabled

    val intake = FalconMAX(30, CANSparkMaxLowLevel.MotorType.kBrushless, DefaultNativeUnitModel)

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
        //intake.setDutyCycle(.30)
    }

    override fun robotPeriodic() {
        Autonomous.update()
        Controls.update()
        Network.update()
    }

    override fun disabledInit() {
    }

    override fun teleopInit() {
    }
}

fun main() {
    Robot.start()
}
