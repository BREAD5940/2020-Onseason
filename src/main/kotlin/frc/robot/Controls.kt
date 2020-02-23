package frc.robot

// import frc.robot.subsystems.drive.VisionDriveCommand
import edu.wpi.first.wpilibj.XboxController
import frc.robot.subsystems.climb.OpenLoopClimbCommand
import frc.robot.subsystems.climb.openLoopClimbCommandGroup
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.drive.HoldAngleCommand
import frc.robot.subsystems.drive.VisionDriveCommand
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.shooter.FlywheelSubsystem
import frc.robot.subsystems.shooter.HoodSubsystem
import frc.robot.subsystems.shooter.ShootCommand
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.wrappers.hid.*

object Controls {

    var isClimbing = false
    val driverWpiXbox = XboxController(0)
    val driverFalconXbox = driverWpiXbox.mapControls {
        val reZeroCommand = { DriveSubsystem.setGyroAngle(0.degrees.toRotation2d()) }
        // button(kBumperLeft).changeOn(reZeroCommand)
        button(kStart).changeOn(reZeroCommand)
        //   state({ !isClimbng }) {

        // todo stuff

//        button(kBumperRight).changeOn { IntakeSubsystem.toggleIntakeExtensionCommand() }
//        button(kBumperLeft).changeOn { FlywheelSubsystem.kickWheelMotor.setDutyCycle(.8) }.changeOff { FlywheelSubsystem.kickWheelMotor.setDutyCycle(0.0) }
        button(kA).changeOn(IntakeSubsystem.extendIntakeCommand())
//        button(kB).changeOn(IntakeSubsystem.retractIntakeCommand())
        button(kX).changeOn { IntakeSubsystem.miniRetractIntakeCommand() }
//        button(kY).change(runCommand(FlywheelSubsystem) { FlywheelSubsystem.shootAtPower(1.0) }).changeOff{FlywheelSubsystem.setNeutral()}

        pov(0).changeOn { HoodSubsystem.wantedAngle = 42.degrees }
        pov(180).changeOn { HoodSubsystem.wantedAngle = 60.degrees }

        pov(270).change(VisionDriveCommand())

        button(kBumperRight).change(ShootCommand().alongWith(VisionDriveCommand()))
        button(kBumperLeft).changeOn { FlywheelSubsystem.kickWheelMotor.setDutyCycle(.8) }
                .changeOff { FlywheelSubsystem.kickWheelMotor.setDutyCycle(0.0) }
//                .change(HoldAngleCommand().alongWith(ShootCommand()))
        button(kB).changeOn { FlywheelSubsystem.kickWheelMotor.setDutyCycle(-0.5) }
                .changeOff { FlywheelSubsystem.kickWheelMotor.setNeutral() }

        button(kStickRight).change(ShootCommand({ Constants.rightBelowGoalParameter5v }))//.alongWith(VisionDriveCommand()))

        button(kY).change(ShootCommand().alongWith(VisionDriveCommand()))

        // button(kB).changeOn{  }
        // }
    }

    val operatorXbox = XboxController(1)
    val operatorFalconXbox = operatorXbox.mapControls {

        button(kBumperRight).change(ShootCommand().alongWith(VisionDriveCommand()))
        button(kBumperLeft).changeOn { FlywheelSubsystem.kickWheelMotor.setDutyCycle(.8) }.changeOff { FlywheelSubsystem.kickWheelMotor.setDutyCycle(0.0) }
        button(kB).changeOn { FlywheelSubsystem.kickWheelMotor.setDutyCycle(-0.5) }.changeOff { FlywheelSubsystem.kickWheelMotor.setNeutral() }
        button(kX).changeOn(IntakeSubsystem.extendIntakeCommand())
        button(kY).change(IntakeSubsystem.retractIntakeCommand())

//        button(kBumperLeft).change(ShootCommand().alongWith(VisionDriveCommand()))
//        button(kBumperRight).changeOn { FlywheelSubsystem.kickWheelMotor.setDutyCycle(.8) }.changeOff { FlywheelSubsystem.kickWheelMotor.setDutyCycle(0.0) }
        button(kA).whileOn { IntakeSubsystem.holdIntake = true }.whileOff { IntakeSubsystem.holdIntake = false }

//        pov(0).changeOn{FlywheelSubsystem.shootAtPower(1.0)}.changeOff { FlywheelSubsystem.setNeutral() }
//        pov(180).changeOn{FlywheelSubsystem.shootAtPower(0.60)}.changeOff { FlywheelSubsystem.setNeutral() }

        pov(0).changeOn(openLoopClimbCommandGroup)

        pov(180).changeOn(OpenLoopClimbCommand().perpetually())

//        pov(180).changeOn { FlywheelSubsystem.shooterMaster.encoder.resetPosition(0.radians) }

//        pov(180).whileOn { FlywheelSubsystem.engagePawl(); println("on ${FlywheelSubsystem.pawlServo.angle}") }
//        pov(0).whileOn { FlywheelSubsystem.disengagePawl(); println("off ${FlywheelSubsystem.pawlServo.angle}") }

        button(kStickRight).change(ShootCommand({ Constants.rightBelowGoalParameter5v }))//.alongWith(VisionDriveCommand()))
    }

    fun update() {
        driverFalconXbox.update()
        operatorFalconXbox.update()
    }
}

private operator fun Boolean.invoke(b: Boolean) {
}

// private fun Command.andThen(block: () -> Unit) = sequential { +this@andThen ; +InstantCommand(Runnable(block)) }
