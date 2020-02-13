package frc.robot

// import frc.robot.subsystems.drive.VisionDriveCommand
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.system.plant.FlywheelSystem
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.shooter.FlywheelSubsystem
import frc.robot.subsystems.shooter.HoodSubsystem
import lib.instantCommand
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.wrappers.hid.*

object Controls {

    var isClimbing = false

    val driverWpiXbox = XboxController(0)
    val driverFalconXbox = driverWpiXbox.mapControls {
        val reZeroCommand = { DriveSubsystem.setGyroAngle(0.degrees.toRotation2d()) }
        //button(kBumperLeft).changeOn(reZeroCommand)
        button(kStart).changeOn(reZeroCommand)

     //   state({ !isClimbing }) {

            // todo stuff

            button(kBumperRight).changeOn{IntakeSubsystem.toggleIntakeExtensionCommand()}
            button(kBumperLeft).changeOn{FlywheelSubsystem.kickWheelMotor.setDutyCycle(1.0)}.changeOff{FlywheelSubsystem.kickWheelMotor.setDutyCycle(0.0)}
            button(kA).changeOn(IntakeSubsystem.extendIntakeCommand())
            button(kB).changeOn(IntakeSubsystem.retractIntakeCommand())
            button(kX).changeOn{IntakeSubsystem.miniExtendIntakeCommand()}
            button(kY).changeOn{FlywheelSubsystem.wantsShootMode = true; FlywheelSubsystem.shooterMaster.setDutyCycle(1.0)}.changeOff{FlywheelSubsystem.shooterMaster.setNeutral()}
            //button(kB).changeOn{  }
       // }
    }

    val operatorXbox = XboxController(1)
    val operatorFalconXbox = operatorXbox.mapControls {

        button(kBumperRight).changeOn{IntakeSubsystem.toggleIntakeExtensionCommand()}
        button(kBumperLeft).changeOn{FlywheelSubsystem.kickWheelMotor.setDutyCycle(1.0)}.changeOff{FlywheelSubsystem.kickWheelMotor.setDutyCycle(0.0)}
        button(kA).changeOn(IntakeSubsystem.extendIntakeCommand())
        button(kB).changeOn(IntakeSubsystem.retractIntakeCommand())
        button(kX).changeOn{IntakeSubsystem.miniExtendIntakeCommand()}
        button(kY).changeOn{FlywheelSubsystem.wantsShootMode = true; FlywheelSubsystem.shooterMaster.setDutyCycle(1.0)}.changeOff{FlywheelSubsystem.shooterMaster.setNeutral()}
        //button(kB).changeOn{  }
    }

    fun update() {
        driverFalconXbox.update()
        operatorFalconXbox.update()
    }
}

private operator fun Boolean.invoke(b: Boolean) {

}

// private fun Command.andThen(block: () -> Unit) = sequential { +this@andThen ; +InstantCommand(Runnable(block)) }
