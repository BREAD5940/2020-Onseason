package frc.robot

// import frc.robot.subsystems.drive.VisionDriveCommand
import edu.wpi.first.wpilibj.XboxController
import frc.robot.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.wrappers.hid.button
import org.ghrobotics.lib.wrappers.hid.kBumperLeft
import org.ghrobotics.lib.wrappers.hid.kStart
import org.ghrobotics.lib.wrappers.hid.mapControls

object Controls {

    var isClimbing = false

    val driverWpiXbox = XboxController(0)
    val driverFalconXbox = driverWpiXbox.mapControls {
        val reZeroCommand = { DriveSubsystem.setGyroAngle(0.degrees.toRotation2d()) }
        button(kBumperLeft).changeOn(reZeroCommand)
        button(kStart).changeOn(reZeroCommand)

        state({ !isClimbing }) {
            // todo stuff
        }
    }

    val operatorXbox = XboxController(1)
    val operatorFalconXbox = driverWpiXbox.mapControls {
        state({ !isClimbing }) {
            //button(kBumperLeft).changeOn { IntakeSubsystem.wantsExtended = false }
            //button(kBumperRight).changeOn { IntakeSubsystem.wantsExtended = true }

            //button(kA).changeOn(FlywheelSubsystem.agitateAndShoot(3000.revolutionsPerMinute))

            state({ operatorXbox.getRawButton(11) }) {
                button(12).changeOn {
                    isClimbing = true
                    //GrabBumperCommand().schedule()
                }
            }
            pov(180).changeOn { isClimbing = false }
        }
    }

    fun update() {
        driverFalconXbox.update()
        operatorFalconXbox.update()
    }
}

// private fun Command.andThen(block: () -> Unit) = sequential { +this@andThen ; +InstantCommand(Runnable(block)) }
