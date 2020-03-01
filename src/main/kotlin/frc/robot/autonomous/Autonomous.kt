package frc.robot.autonomous

import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.Network
import frc.robot.Robot
import frc.robot.autonomous.routines.*
import org.ghrobotics.lib.utils.monitor
import org.ghrobotics.lib.utils.onChangeToTrue
import org.ghrobotics.lib.wrappers.FalconTimedRobot

/**
 * Manages the autonomous mode of the game.
 */
object Autonomous {

    // Auto mode to run
    private val autoMode = { Network.autoModeChooser.selected }

    val autoModeMonitor = autoMode.monitor

    // Stores if we are ready to send it.
    @Suppress("MemberVisibilityCanBePrivate")
    private val isReady =
            { Robot.currentMode == FalconTimedRobot.Mode.AUTONOMOUS && Robot.isEnabled }

    // Update the autonomous listener.
    fun update() {
        // Update localization.
//        autoModeMonitor.onChange {
//            if (!Robot.isEnabled) DriveSubsystem.odometry.resetPosition(it.pose, DriveSubsystem.periodicIO.pose.rotation)
//        }

        // update our selected auto mode
        selectedAutonomous = possibleAutos[autoMode()] ?: doNothing

        @Suppress("UNUSED_ANONYMOUS_PARAMETER")
        robotModeMonitor.onChange { newValue ->
            if (newValue == FalconTimedRobot.Mode.TELEOP) selectedAutonomous.cancel()
        }

        isReadyMonitor.onChangeToTrue {
            JUST S3ND IT
        }
    }

    private val possibleAutos = hashMapOf(
            Mode.EIGHT_PC_TRENCH to EightPCFromTrenchRoutine()(),
            Mode.THREE_PC to ThreePCRoutine()(),
            Mode.SIX_PC_TRENCH to SixPCFromTrenchRoutine()(),
            Mode.OPPOSING_TRENCH to OpposingTrenchRoutine()(),
            Mode.TEN_PC_RIGHT to RightSideTenPCRoutine()(),
            Mode.DO_NOTHING to InstantCommand()
    )
    private var selectedAutonomous: CommandBase = InstantCommand()
    private val doNothing = InstantCommand()

    private val JUST get() = selectedAutonomous

    private fun startAuto() {
        selectedAutonomous.schedule()
    }

    @Suppress("LocalVariableName")
    private val IT = ""

    private val isReadyMonitor = isReady.monitor
    private val robotModeMonitor = { Robot.currentMode }.monitor

    enum class Mode { THREE_PC, SIX_PC_TRENCH, EIGHT_PC_TRENCH, OPPOSING_TRENCH, TEN_PC_RIGHT, DO_NOTHING }
}

@Suppress("FunctionName")
private infix fun CommandBase.S3ND(it: Any) = this@S3ND.schedule()
