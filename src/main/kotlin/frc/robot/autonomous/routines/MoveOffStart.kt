package frc.robot.autonomous.routines

import frc.robot.auto.paths.TrajectoryFactory
import frc.robot.subsystems.drive.DriveSubsystem
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.mathematics.units.derived.degrees

class MoveOffStart : AutoRoutine() {
    private val path1 = TrajectoryFactory.moveForward5Feet

    override val duration: SIUnit<Second>
        get() = path1.duration

    override val routine
        get() = sequential {
            +DriveSubsystem.followTrajectory2(path1) { 0.0.degrees }
        }
}
