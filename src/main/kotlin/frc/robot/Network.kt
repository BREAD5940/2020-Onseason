/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package frc.robot

import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import frc.robot.autonomous.Autonomous
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.shooter.FlywheelSubsystem
import frc.robot.subsystems.shooter.HoodSubsystem
import lib.inRpm
import org.ghrobotics.lib.mathematics.units.derived.inDegrees
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.wrappers.networktables.enumSendableChooser
import org.ghrobotics.lib.wrappers.networktables.tab
import kotlin.math.roundToInt

object Network {
    fun update() {
    }

    val startingPositionChooser = enumSendableChooser<Autonomous.StartingPositions>()
    val autoModeChooser = enumSendableChooser<Autonomous.Mode>()

    private val mainShuffleboardDisplay: ShuffleboardTab = Shuffleboard.getTab("OBINAN")

    private val autoLayout = mainShuffleboardDisplay.getLayout("Autonomous", BuiltInLayouts.kList)
            .withPosition(0, 0)
            .withSize(2, 2)
    init {

        val table = tab("OBINAN") {
            list("Angles") {
                number("fl") { DriveSubsystem.flModule.state.angle.degrees.roundToInt().toDouble() }
                number("fr") { DriveSubsystem.frModule.state.angle.degrees.roundToInt().toDouble() }
                number("bl") { DriveSubsystem.blModule.state.angle.degrees.roundToInt().toDouble() }
                number("br") { DriveSubsystem.brModule.state.angle.degrees.roundToInt().toDouble() }
                position(3, 0)
                size(1, 3)
            }
            list("Flywheel") {
                number("Velocity, RPM") { FlywheelSubsystem.shooterMaster.encoder.velocity.inRpm() }
                number("Output, V") { FlywheelSubsystem.shooterMaster.voltageOutput.value }
                number("Hood Angle, Deg") { HoodSubsystem.hoodAngle.inDegrees().roundToInt().toDouble() }
                number("Hood Error, Deg") { HoodSubsystem.hoodPidController.positionError.radians.inDegrees().roundToInt().toDouble() }
                number("Hood Output, Volt") { HoodSubsystem.hoodMotor.voltageOutput.value }
                number("Last ref pos, deg") { HoodSubsystem.lastProfiledReference.position.radians.inDegrees().roundToInt().toDouble() }
                position(2, 0)
                size(1, 4)
            }
        }

        startingPositionChooser.setDefaultOption(Autonomous.StartingPositions.LEFT.name, Autonomous.StartingPositions.LEFT)
        autoModeChooser.setDefaultOption(Autonomous.Mode.DO_NOTHING.name, Autonomous.Mode.DO_NOTHING)

        // Put choosers on dashboard
        autoLayout.add("Auto Mode", autoModeChooser)
        autoLayout.add("Starting Position", startingPositionChooser)
    }
}
