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
import org.ghrobotics.lib.wrappers.networktables.enumSendableChooser
import org.ghrobotics.lib.wrappers.networktables.tab

object Network {

    val startingPositionChooser = enumSendableChooser<Autonomous.StartingPositions>()
    val autoModeChooser = enumSendableChooser<Autonomous.Mode>()

    private val mainShuffleboardDisplay: ShuffleboardTab = Shuffleboard.getTab("CROISSANT")

    private val autoLayout = mainShuffleboardDisplay.getLayout("Autonomous", BuiltInLayouts.kList)
            .withPosition(0, 0)
            .withSize(2, 2)
    private val driveSubsystemLayout = mainShuffleboardDisplay.getLayout("Drive", BuiltInLayouts.kGrid)
            .withPosition(4, 0)
            .withSize(4, 3)

    private val flAngle = driveSubsystemLayout.add("FL Angle (deg)", 0.0).entry
    private val frAngle = driveSubsystemLayout.add("FR Angle (deg)", 0.0).entry
    private val brAngle = driveSubsystemLayout.add("BR Angle (deg)", 0.0).entry
    private val blAngle = driveSubsystemLayout.add("BL Angle (deg)", 0.0).entry

    private val flAzimuthOutput = driveSubsystemLayout.add("FL Azumith Output (volts)", 0.0).entry
    private val frAzimuthOutput = driveSubsystemLayout.add("FR Azumith Output (volts)", 0.0).entry
    private val brAzimuthOutput = driveSubsystemLayout.add("BR Azumith Output (volts)", 0.0).entry
    private val blAzimuthOutput = driveSubsystemLayout.add("BL Azumith Output (volts)", 0.0).entry

    private val flAzimuthError = driveSubsystemLayout.add("FL Error (deg)", 0.0).entry
    private val frAzimuthError = driveSubsystemLayout.add("FR Error (deg)", 0.0).entry
    private val brAzimuthError = driveSubsystemLayout.add("BR Error (deg)", 0.0).entry
    private val blAzimuthError = driveSubsystemLayout.add("BL Error (deg)", 0.0).entry

    init {

        val table = tab("croissant") {
            textView("robot X", { DriveSubsystem.robotPosition.translation.x }) {
                position(0, 0)
                size(1, 1)
            }.withWidget(BuiltInWidgets.kGraph)
        }

        startingPositionChooser.setDefaultOption(Autonomous.StartingPositions.LEFT.name, Autonomous.StartingPositions.LEFT)
        autoModeChooser.setDefaultOption(Autonomous.Mode.DO_NOTHING.name, Autonomous.Mode.DO_NOTHING)

        // Put choosers on dashboard
        autoLayout.add("Auto Mode", autoModeChooser)
        autoLayout.add("Starting Position", startingPositionChooser)
    }

    fun update() {
        with(DriveSubsystem) {
            flAngle.setDouble(flModule.state.angle.degrees)
            frAngle.setDouble(frModule.state.angle.degrees)
            brAngle.setDouble(brModule.state.angle.degrees)
            blAngle.setDouble(blModule.state.angle.degrees)

            flAzimuthError.setDouble(flModule.periodicIO.lastError.degrees)
            frAzimuthError.setDouble(frModule.periodicIO.lastError.degrees)
            brAzimuthError.setDouble(brModule.periodicIO.lastError.degrees)
            blAzimuthError.setDouble(blModule.periodicIO.lastError.degrees)

            val volts = RobotController.getBatteryVoltage()
            flAzimuthOutput.setDouble(flModule.periodicIO.lastAzimuthOutput * volts)
            frAzimuthOutput.setDouble(frModule.periodicIO.lastAzimuthOutput * volts)
            brAzimuthOutput.setDouble(brModule.periodicIO.lastAzimuthOutput * volts)
            blAzimuthOutput.setDouble(blModule.periodicIO.lastAzimuthOutput * volts)
        }
    }
}
