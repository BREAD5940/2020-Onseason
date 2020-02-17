package frc.robot

// import frc.robot.subsystems.drive.VisionDriveCommand
import edu.wpi.first.wpilibj.XboxController
import frc.robot.subsystems.drive.DriveSubsystem
<<<<<<< HEAD
<<<<<<< HEAD
import frc.robot.subsystems.drive.HoldAngleCommand
import frc.robot.subsystems.drive.HolomonicDriveCommand
=======
>>>>>>> parent of e9bde76... use gyro when shooting
=======
>>>>>>> parent of e9bde76... use gyro when shooting
import frc.robot.subsystems.drive.VisionDriveCommand
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.shooter.FlywheelSubsystem
import frc.robot.subsystems.shooter.HoodSubsystem
import frc.robot.subsystems.shooter.ShootCommand
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.wrappers.hid.*

object Controls {

    var isClimbing = false
    var isRobotRelative = false
    val driverWpiXbox = XboxController(0)
    val driverFalconXbox = driverWpiXbox.mapControls {
        val reZeroCommand = { DriveSubsystem.setGyroAngle(0.degrees.toRotation2d()) }
        // button(kBumperLeft).changeOn(reZeroCommand)
        button(kStart).changeOn(reZeroCommand)
        button(kStickLeft).changeOn{isRobotRelative = true}.whileOff{ isRobotRelative = false}
        //   state({ !isClimbng }) {

        // todo stuff

<<<<<<< HEAD
        //  button(kBumperRight).changeOn{IntakeSubsystem.toggleIntakeExtensionCommand()}
        // button(kBumperLeft).changeOn{FlywheelSubsystem.kickWheelMotor.setDutyCycle(.8)}.changeOff{FlywheelSubsystem.kickWheelMotor.setDutyCycle(0.0)}
        button(kY).changeOn(IntakeSubsystem.extendIntakeCommand())
        button(kX).changeOn(IntakeSubsystem.retractIntakeCommand())
        // button(kX).changeOn{IntakeSubsystem.miniRetractIntakeCommand()}
=======
        button(kBumperRight).changeOn{IntakeSubsystem.toggleIntakeExtensionCommand()}
        button(kBumperLeft).changeOn{FlywheelSubsystem.kickWheelMotor.setDutyCycle(.8)}.changeOff{FlywheelSubsystem.kickWheelMotor.setDutyCycle(0.0)}
        button(kA).changeOn(IntakeSubsystem.extendIntakeCommand())
        button(kB).changeOn(IntakeSubsystem.retractIntakeCommand())
        button(kX).changeOn{IntakeSubsystem.miniRetractIntakeCommand()}
>>>>>>> parent of 6be6d64... Max messing with driver/operator controls
//        button(kY).change(runCommand(FlywheelSubsystem) { FlywheelSubsystem.shootAtPower(1.0) }).changeOff{FlywheelSubsystem.setNeutral()}

        pov(0).changeOn { HoodSubsystem.wantedAngle = 42.degrees }
        pov(180).changeOn { HoodSubsystem.wantedAngle = 60.degrees }

        pov(270).change(VisionDriveCommand())

<<<<<<< HEAD
        button(kBumperRight).change(ShootCommand().alongWith(VisionDriveCommand()))
        button(kBumperLeft).changeOn { FlywheelSubsystem.kickWheelMotor.setDutyCycle(.8) }.changeOff { FlywheelSubsystem.kickWheelMotor.setDutyCycle(0.0) }
        button(kB).changeOn { FlywheelSubsystem.kickWheelMotor.setDutyCycle(-0.5) }.changeOff { FlywheelSubsystem.kickWheelMotor.setNeutral(); FlywheelSubsystem.wantsShootMode = false }

        button(kStickRight).change(ShootCommand({ Constants.rightBelowGoalParameter }))//.alongWith(VisionDriveCommand()))
=======
        button(kY).change(ShootCommand().alongWith(VisionDriveCommand()))

        //button(kB).changeOn{  }
        // }
>>>>>>> parent of 6be6d64... Max messing with driver/operator controls
    }

    val operatorXbox = XboxController(1)
    val operatorFalconXbox = operatorXbox.mapControls {

<<<<<<< HEAD
        button(kBumperRight).change(ShootCommand().alongWith(VisionDriveCommand()))
        button(kBumperLeft).changeOn { FlywheelSubsystem.kickWheelMotor.setDutyCycle(.8) }.changeOff { FlywheelSubsystem.kickWheelMotor.setDutyCycle(0.0) }
        button(kB).changeOn { FlywheelSubsystem.kickWheelMotor.setDutyCycle(-0.5) }.changeOff { FlywheelSubsystem.kickWheelMotor.setNeutral(); FlywheelSubsystem.wantsShootMode = false }
        button(kX).changeOn(IntakeSubsystem.extendIntakeCommand())
        button(kY).change(ShootCommand().alongWith(VisionDriveCommand()))
=======
        button(kBumperLeft).change(ShootCommand().alongWith(VisionDriveCommand()))
        button(kBumperRight).changeOn {FlywheelSubsystem.kickWheelMotor.setDutyCycle(.8)}.changeOff{FlywheelSubsystem.kickWheelMotor.setDutyCycle(0.0)}

        button(kB).changeOn{IntakeSubsystem.intakeMotor.setDutyCycle(-0.5); FlywheelSubsystem.kickWheelMotor.setDutyCycle(-0.5)}.changeOff{IntakeSubsystem.intakeMotor.setNeutral(); FlywheelSubsystem.kickWheelMotor.setNeutral(); FlywheelSubsystem.wantsShootMode = false}
        button(kA).whileOn{IntakeSubsystem.holdIntake = true}.whileOff{IntakeSubsystem.holdIntake = false}

//        pov(0).changeOn{FlywheelSubsystem.shootAtPower(1.0)}.changeOff { FlywheelSubsystem.setNeutral() }
//        pov(180).changeOn{FlywheelSubsystem.shootAtPower(0.60)}.changeOff { FlywheelSubsystem.setNeutral() }

>>>>>>> parent of 6be6d64... Max messing with driver/operator controls
        pov(0).change(ShootCommand())
        // todo make climb shit

<<<<<<< HEAD
        button(kStickRight).change(ShootCommand({ Constants.rightBelowGoalParameter }))//.alongWith(VisionDriveCommand()))
=======
>>>>>>> parent of 6be6d64... Max messing with driver/operator controls
    }

    fun update() {
        driverFalconXbox.update()
        operatorFalconXbox.update()
    }
}

private operator fun Boolean.invoke(b: Boolean) {
}

// private fun Command.andThen(block: () -> Unit) = sequential { +this@andThen ; +InstantCommand(Runnable(block)) }
