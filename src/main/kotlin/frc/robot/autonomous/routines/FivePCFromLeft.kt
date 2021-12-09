package frc.robot.autonomous.routines

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Pose2d
import frc.robot.auto.paths.TrajectoryFactory
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.drive.VisionDriveCommand
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.shooter.FlywheelSubsystem
import lib.beforeStarting
import lib.instantCommand
import lib.runCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.mathematics.units.seconds

class FivePCFromLeft : AutoRoutine() {
    private val path1 = TrajectoryFactory.fivePCToTrench
    private val path2 = TrajectoryFactory.opposingTrenchToShoot

    override val duration: SIUnit<Second>
        get() = SIUnit<Second>(path1.totalTimeSeconds)

    override val routine
        get() = sequential {
            +instantCommand { DriveSubsystem.robotPosition = Pose2d(path1.states.first().poseMeters.translation, 180.degrees.toRotation2d()) }

            +DriveSubsystem.followTrajectory2(path1) { 180.degrees }
                .deadlineWith(
                    IntakeSubsystem.extendIntakeCommand()
                        .andThen(runCommand(IntakeSubsystem) { IntakeSubsystem.setSpeed(1.0) })
                )

            val timer = Timer()
            +DriveSubsystem.followTrajectory(path2) { if (timer.get() > 0.5) 0.degrees.toRotation2d() else 180.degrees.toRotation2d() }
                .beforeStarting(Runnable { IntakeSubsystem.setNeutral(); timer.reset(); timer.start() }, IntakeSubsystem)

            +(FlywheelSubsystem.agitateAndShoot((4.seconds)))
                .deadlineWith(VisionDriveCommand())
        }
}
