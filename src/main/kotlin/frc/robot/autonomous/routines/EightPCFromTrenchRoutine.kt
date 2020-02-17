package frc.robot.autonomous.routines

import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.robot.auto.paths.TrajectoryFactory
import frc.robot.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second

class EightPCFromTrenchRoutine : AutoRoutine() {
    private val path1 = TrajectoryFactory.shootThree // three we have
    private val path2 = TrajectoryFactory.getPCFromTrench // grab 5 from trench and drive to shooting position
    private val path3 = TrajectoryFactory.trenchToShoot

    override val duration: SIUnit<Second>
        get() = SIUnit<Second>(path1.totalTimeSeconds + path2.totalTimeSeconds + path3.totalTimeSeconds)
    override val routine
        get() = sequential {
            // if intake isn't automatic this will need to be refactored to run the intake
            +PrintCommand("Starting Autonomous Routine")
            +DriveSubsystem.followTrajectory(
                    path1,
                    path1.states.last().poseMeters.rotation
            )

            // shoot three balls here

            +DriveSubsystem.followTrajectory(
                    path2,
                    path2.states.last().poseMeters.rotation
            )

            // shoot five balls here
        }
    // probably more stuff here but idk
}
