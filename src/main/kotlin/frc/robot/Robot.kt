
package frc.robot

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.StartEndCommand
import frc.robot.autonomous.Autonomous
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.shooter.FlywheelSubsystem
import org.ghrobotics.lib.wrappers.FalconTimedRobot

object Robot : FalconTimedRobot() {

    val isEnabled get() = wrappedValue.isEnabled

    // val intake = FalconMAX(11, CANSparkMaxLowLevel.MotorType.kBrushless, DefaultNativeUnitModel)

    override fun robotInit() {
        Network // at the top because s3ndable choosers need to be instantiated
        Autonomous

        // + for subsystems
        +DriveSubsystem
            //+FlywheelSubsystem
        //+BumperGrabberSubsystem
        +IntakeSubsystem
        //+VisionSubsystem

        SmartDashboard.putData(CommandScheduler.getInstance())

        SmartDashboard.putData("agitate", StartEndCommand(Runnable { FlywheelSubsystem.runKickWheel(1.0) }, Runnable { FlywheelSubsystem.setNeutral() }, FlywheelSubsystem))

        super.robotInit()
    }


    override fun teleopPeriodic() {
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


    override fun autonomousInit() {
    }

}

fun main() {
    Robot.start()
}
