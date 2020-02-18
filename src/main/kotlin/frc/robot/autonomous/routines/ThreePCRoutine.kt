package frc.robot.autonomous.routines

import frc.robot.auto.paths.TrajectoryFactory
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.drive.VisionDriveCommand
import frc.robot.subsystems.shooter.FlywheelSubsystem
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.mathematics.units.seconds

class ThreePCRoutine : AutoRoutine() {
    private val path1 = TrajectoryFactory.moveForward5Feet

    override val duration: SIUnit<Second>
        get() = SIUnit<Second>(path1.totalTimeSeconds)
    override val routine
        get() = sequential {
            +DriveSubsystem.followTrajectory(path1) { 180.0.degrees.toRotation2d() }

            +(FlywheelSubsystem.agitateAndShoot((4.seconds)))
                    .deadlineWith(VisionDriveCommand())
        }
}
