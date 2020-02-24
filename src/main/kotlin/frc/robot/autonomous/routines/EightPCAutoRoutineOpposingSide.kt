package frc.robot.autonomous.routines

import edu.wpi.first.wpilibj.geometry.Pose2d
import frc.robot.auto.paths.TrajectoryFactory
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.drive.PointTurnCommand
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

class EightPCAutoRoutineOpposingSide : AutoRoutine() {
    private val path1 = TrajectoryFactory.eightPCAutoStartToOpposingTrench
    private val path2 = TrajectoryFactory.eightPCAutoOpposingTrenchToShoot
    private val path3 = TrajectoryFactory.eightPCAutoShootToShieldGenerator
    private val path4 = TrajectoryFactory.eightPCShieldGeneratorToShoot

    override val duration: SIUnit<Second>
        get() = path1.duration +
                path2.duration +
                path3.duration +
                path4.duration

    override val routine
        get() = sequential {
            +instantCommand { DriveSubsystem.robotPosition = Pose2d(path1.states.first().poseMeters.translation, 0.degrees.toRotation2d()) }

            +DriveSubsystem.followTrajectory2(path1) { 0.0.degrees }
                    .deadlineWith(
                            IntakeSubsystem.extendIntakeCommand()
                                    .andThen(runCommand(IntakeSubsystem) { IntakeSubsystem.setSpeed(1.0) }))
                    .andThen(Runnable { IntakeSubsystem.setNeutral() }, IntakeSubsystem)

            +PointTurnCommand(160.degrees.toRotation2d())

            +DriveSubsystem.followTrajectory(path2) { (160.0).degrees.toRotation2d() }

            +PointTurnCommand(24.degrees.toRotation2d())

            +(FlywheelSubsystem.agitateAndShoot((3.seconds)))
                    .deadlineWith(VisionDriveCommand())

            +DriveSubsystem.followTrajectory(path3) { 24.0.degrees.toRotation2d() }
                    .deadlineWith(runCommand(IntakeSubsystem) { IntakeSubsystem.setSpeed(1.0) })
                    .andThen(Runnable { IntakeSubsystem.setNeutral() }, IntakeSubsystem)

            +PointTurnCommand(160.degrees.toRotation2d())

            +DriveSubsystem.followTrajectory(path4) { 160.0.degrees.toRotation2d() }

            +(FlywheelSubsystem.agitateAndShoot((3.seconds)))
                    .deadlineWith(
                            VisionDriveCommand(),
                            runCommand(IntakeSubsystem) { IntakeSubsystem.setSpeed(1.0); IntakeSubsystem.setSmolPistonExtension(true) })
                    .andThen(Runnable { IntakeSubsystem.setNeutral() }, IntakeSubsystem)
        }
}
