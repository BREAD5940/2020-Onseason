package frc.robot.autonomous.routines

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.trajectory.Trajectory
import edu.wpi.first.wpilibj2.command.*
import frc.robot.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.utils.BooleanSource
import org.ghrobotics.lib.utils.Source

abstract class AutoRoutine : SequentialCommandGroup(), Source<Command> {

    abstract val duration: SIUnit<Second>
    abstract val routine: Command

    override fun invoke() = sequential {
        +InstantCommand(Runnable {
            println("[AutoRoutine] Starting routine...")
        })
        +routine
    }.raceWith(WaitUntilCommand { /*Robot.emergencyActive*/ false }) as CommandBase

    fun followVisionAssistedTrajectory(
        originalTrajectory: Trajectory,
        pathMirrored: BooleanSource,
        radiusFromEnd: SIUnit<Meter>,
        useAbsoluteVision: Boolean = false
    ): CommandBase = InstantCommand(Runnable { TODO("aha") })
//            "VisionAssistedTrajectoryTracker(
//            pathMirrored.map(originalTrajectory.mirror(), originalTrajectory),
//            radiusFromEnd,
//            useAbsoluteVision
//    )

    protected fun executeFor(time: SIUnit<Second>, command: FalconCommand) = sequential {
        +command
        +WaitCommand(100.0)
    }.withTimeout(time)

    private fun Pose2d.asString() = "Pose X:${translation.x / kFeetToMeter}\' Y:${translation.y / kFeetToMeter}' Theta:${rotation.degrees}deg"

    fun notWithinRegion(region: Rectangle2d) = object : CommandBase() {
        override fun isFinished() = !region.contains(DriveSubsystem.robotPosition.translation)
    }

    operator fun Command.unaryPlus() {
        addCommands(this@unaryPlus)
    }
}

fun Command.withExit(exit: BooleanSource): Command = this.raceWith(WaitUntilCommand(exit))

fun Command.withTimeout(second: SIUnit<Second>): Command = this.withTimeout(second.second)

val Trajectory.duration get() = totalTimeSeconds.seconds
