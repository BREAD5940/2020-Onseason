package frc.robot.autonomous.routines

import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.robot.auto.paths.TrajectoryFactory
import frc.robot.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second

class ThreePCRoutine : AutoRoutine() {
    private val path1 = TrajectoryFactory.grabThreeFromTrench // three we have

    override val duration: SIUnit<Second>
        get() = SIUnit<Second>(path1.totalTimeSeconds)
    override val routine
        get() = sequential {
            // if intake isn't automatic this will need to be refactored to run the intake
            +PrintCommand("Starting Autonomous Routine")
            +DriveSubsystem.followTrajectory(
                    path1,
                    path1.states.last().poseMeters.rotation
            )

            // shoot three balls here
        }
    // probably more stuff here but idk
}
