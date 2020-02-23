package frc.robot.autonomous.routines

import edu.wpi.first.wpilibj.geometry.Pose2d
import frc.robot.auto.paths.TrajectoryFactory
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.drive.VisionDriveCommand
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.shooter.FlywheelSubsystem
import lib.instantCommand
import lib.runCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.mathematics.units.seconds

class EightPCFromTrenchRoutine : AutoRoutine() {
    private val path1 = TrajectoryFactory.shootThree
    private val path2 = TrajectoryFactory.getPCFromTrench
    private val path3 = TrajectoryFactory.trenchToShoot

    override val duration: SIUnit<Second>
        get() = SIUnit<Second>(path1.totalTimeSeconds + path2.totalTimeSeconds + path3.totalTimeSeconds)

    override val routine
        get() = sequential {

            +instantCommand { DriveSubsystem.robotPosition = Pose2d(path1.states.first().poseMeters.translation, 180.degrees.toRotation2d()) }

            +DriveSubsystem.followTrajectory(path1) { 160.0.degrees.toRotation2d() }

            +(FlywheelSubsystem.agitateAndShoot(4.seconds))
                    .deadlineWith(VisionDriveCommand())

            +DriveSubsystem.followTrajectory2(path2) { (0.0).degrees }
                    .alongWith(
                    IntakeSubsystem.extendIntakeCommand()
                            .andThen(runCommand(IntakeSubsystem) { IntakeSubsystem.setSpeed(0.5) }))
                    .andThen(Runnable { IntakeSubsystem.setNeutral() }, IntakeSubsystem)

            +DriveSubsystem.followTrajectory(path3) { 180.0.degrees.toRotation2d() }

            +(FlywheelSubsystem.agitateAndShoot(4.seconds))
                    .deadlineWith(VisionDriveCommand())
        }
}
