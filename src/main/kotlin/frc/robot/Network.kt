/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package frc.robot

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import frc.robot.autonomous.Autonomous
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.shooter.FlywheelSubsystem
import frc.robot.subsystems.shooter.HoodSubsystem
import kotlin.math.roundToInt
import lib.inRpm
import lib.revolutionsPerMinute
import org.ghrobotics.lib.mathematics.units.derived.inDegrees
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.wrappers.networktables.enumSendableChooser
import org.ghrobotics.lib.wrappers.networktables.tab

object Network {
    fun update() {
    }

    val autoModeChooser = enumSendableChooser<Autonomous.Mode>()

    private val mainShuffleboardDisplay: ShuffleboardTab = Shuffleboard.getTab("OBINAN")

    init {

        val table = tab("OBINAN") {

            list("Autonomous") {
                position(0, 0)
                size(2, 2)
                sendable("Auto Mode", autoModeChooser) {
                    autoModeChooser.setDefaultOption(Autonomous.Mode.DO_NOTHING.name, Autonomous.Mode.DO_NOTHING)
                }
            }

            list("Angles") {
                number("fl") { DriveSubsystem.flModule.state.angle.degrees.roundToInt().toDouble() }
                number("fr") { DriveSubsystem.frModule.state.angle.degrees.roundToInt().toDouble() }
                number("bl") { DriveSubsystem.blModule.state.angle.degrees.roundToInt().toDouble() }
                number("br") { DriveSubsystem.brModule.state.angle.degrees.roundToInt().toDouble() }
                position(3, 0)
                size(1, 3)
            }
            list("Flywheel") {
                number("Velocity, RPM") { FlywheelSubsystem.flywheelSpeed.inRpm() }
                number("Output, V") { FlywheelSubsystem.shooterMaster.voltageOutput.value }
                number("Hood Angle, Deg") { HoodSubsystem.hoodAngle.inDegrees().roundToInt().toDouble() }
                number("Hood Error, Deg") { HoodSubsystem.hoodPidController.positionError.radians.inDegrees().roundToInt().toDouble() }
                number("Hood Output, Volt") { HoodSubsystem.hoodMotor.voltageOutput.value }
                number("Last ref pos, deg") { HoodSubsystem.lastProfiledReference.position.radians.inDegrees().roundToInt().toDouble() }
//                number("Shooter error, RPM") { (FlywheelSubsystem.shooterMaster.encoder.velocity - 3000.revolutionsPerMinute).inRpm() }
                number("thru bore pos, deg") { FlywheelSubsystem.throughBoreEncoder.distance.radians.inDegrees()}
                position(2, 0)
                size(1, 4)
            }
        }
    }
}
