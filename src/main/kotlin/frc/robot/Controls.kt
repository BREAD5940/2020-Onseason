package frc.robot

import edu.wpi.first.wpilibj.XboxController
import frc.robot.subsystems.drive.DriveSubsystem
// import frc.robot.subsystems.drive.VisionDriveCommand
import org.ghrobotics.lib.mathematics.units.derived.*
import org.ghrobotics.lib.wrappers.hid.* // ktlint-disable no-wildcard-imports

object Controls {

    private val driverControllerLowLevel = XboxController(0)
    val driverFalconXbox = driverControllerLowLevel.mapControls {

        val reZeroCommand = { DriveSubsystem.setGyroAngle(0.degrees.toRotation2d()) }
        button(kBumperLeft).changeOn(reZeroCommand)
        button(kStart).changeOn(reZeroCommand)
    }

    fun update() {
        driverFalconXbox.update()
    }
}

// private fun Command.andThen(block: () -> Unit) = sequential { +this@andThen ; +InstantCommand(Runnable(block)) }
