package frc.robot

import edu.wpi.first.wpilibj.XboxController
import frc.robot.subsystems.climb.GrabBumperCommand
import frc.robot.subsystems.climb.OpenLoopClimbCommand
import frc.robot.subsystems.climb.openLoopClimbCommandGroup
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.drive.VisionDriveCommand
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.shooter.FlywheelSubsystem
import frc.robot.subsystems.shooter.ShootCommand
import lib.instantCommand
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.wrappers.hid.*

object Controls {

    var isClimbing = false
    val driverWpiXbox = XboxController(0)
    val driverFalconXbox = driverWpiXbox.mapControls {
        val reZeroCommand = { DriveSubsystem.setGyroAngle(0.degrees.toRotation2d()) }

        button(kStart).changeOn(reZeroCommand)
        button(kA).changeOn(IntakeSubsystem.extendIntakeCommand())
        button(kX).changeOn { IntakeSubsystem.miniRetractIntakeCommand() }
        button(kBumperRight).change(ShootCommand().alongWith(VisionDriveCommand()))
        button(kStickRight).change(ShootC        button(kBumperLeft).changeOn { FlywheelSubsystem.kickWheelMotor.setDutyCycle(.8) }.changeOff { FlywheelSubsystem.kickWheelMotor.setNeutral() }
                ommand({ Constants.rightBelowGoalParameter5v }))//.alongWith(VisionDriveCommand()))
        button(kY).change(ShootCommand().alongWith(VisionDriveCommand()))
    }

    val operatorXbox = XboxController(1)
    val operatorFalconXbox = operatorXbox.mapControls {

        button(kBumperRight).change(ShootCommand().alongWith(VisionDriveCommand()))
        button(kBumperLeft).changeOn { FlywheelSubsystem.kickWheelMotor.setDutyCycle(.8) }.changeOff { FlywheelSubsystem.kickWheelMotor.setDutyCycle(0.0) }
        button(kB).changeOn { FlywheelSubsystem.kickWheelMotor.setDutyCycle(-0.5) }.changeOff { FlywheelSubsystem.kickWheelMotor.setNeutral() }
        button(kX).changeOn(IntakeSubsystem.extendIntakeCommand())
        button(kY).change(IntakeSubsystem.retractIntakeCommand())
        pov(0).changeOn(openLoopClimbCommandGroup.alongWith(GrabBumperCommand(), instantCommand(IntakeSubsystem) {}))
//        pov(180).changeOn(OpenLoopClimbCommand().perpetually())
        pov(180).changeOn(GrabBumperCommand().alongWith(instantCommand(IntakeSubsystem) {}))
        button(kStickRight).change(ShootCommand({ Constants.rightBelowGoalParameter5v }))
    }

    fun update() {
        driverFalconXbox.update()
        operatorFalconXbox.update()
    }
}
