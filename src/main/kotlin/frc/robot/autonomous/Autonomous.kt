package frc.robot.autonomous

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.Network
import frc.robot.Robot
import frc.robot.autonomous.paths.TrajectoryWaypoints
import frc.robot.autonomous.routines.*
import frc.robot.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.mathematics.twodim.geometry.mirror
import org.ghrobotics.lib.utils.Source
import org.ghrobotics.lib.utils.and
import org.ghrobotics.lib.utils.monitor
import org.ghrobotics.lib.utils.onChangeToTrue
import org.ghrobotics.lib.wrappers.FalconTimedRobot

/**
 * Manages the autonomous mode of the game.
 */
object Autonomous {

    // Auto mode to run
    private val autoMode = { Network.autoModeChooser.selected }

    // Starting position of the robot
    val startingPosition = { Network.startingPositionChooser.selected }

    // Stores whether the current config is valid.
    private var configValid = Source(true)

    val isStartingOnLeft = { val position = startingPosition()
        position == StartingPositions.LEFT
    }

    // Stores if we are ready to send it.
    private val isReady =
            { Robot.currentMode == FalconTimedRobot.Mode.AUTONOMOUS && Robot.isEnabled } and configValid

    // Update the autonomous listener.
    fun update() {
        // Update localization.
        startingPositionMonitor.onChange { if (!Robot.isEnabled) DriveSubsystem.odometry.resetPosition(it.pose, DriveSubsystem.periodicIO.pose.rotation) }

        // update our selected auto mode
        selectedAutonomous = possibleAutos[autoMode()] ?: doNothing

        @Suppress("UNUSED_ANONYMOUS_PARAMETER")
        robotModeMonitor.onChange { newValue ->
            // maybe stop auto on change to enabled?
        }

        isReadyMonitor.onChangeToTrue {
            startAuto()
        }
    }

    private val possibleAutos = hashMapOf(
            Mode.EIGHT_PC_TRENCH to EightPCFromTrenchRoutine()(),
            Mode.THREE_PC to ThreePCRoutine()(),
//            Mode.EIGHT_PC_SHIELD_GENERATOR to EightPCFromShieldGeneratorRoutine()(),
            Mode.SIX_PC_TRENCH to SixPCFromTrenchRoutine()(),
            Mode.EIGHT_PC_OPPOSING_TRENCH to EightPCAutoRoutineOpposingSide()(),
            Mode.TEN_PC to TenPCAutoRoutine()(),
            Mode.MOVE_OFF_START to MoveOffStart()(),
            Mode.DO_NOTHING to InstantCommand()


    )
    private var selectedAutonomous: CommandBase = InstantCommand()
    private val doNothing = selectedAutonomous

    private fun startAuto() {
        selectedAutonomous.schedule()
    }

    @Suppress("LocalVariableName")
    private val IT = ""

    private val startingPositionMonitor = startingPosition.monitor
    private val isReadyMonitor = isReady.monitor
    private val robotModeMonitor = { Robot.currentMode }.monitor

    enum class StartingPositions(val pose: Pose2d) {
        LEFT(TrajectoryWaypoints.kSideStart),
        RIGHT(TrajectoryWaypoints.kSideStart.mirror()),
    }

    enum class Mode { THREE_PC, SIX_PC_TRENCH, EIGHT_PC_TRENCH, EIGHT_PC_SHIELD_GENERATOR, EIGHT_PC_OPPOSING_TRENCH, TEN_PC, MOVE_OFF_START, DO_NOTHING }
}
