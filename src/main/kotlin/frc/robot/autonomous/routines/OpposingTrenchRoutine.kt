package frc.robot.autonomous.routines

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Pose2d
import frc.robot.auto.paths.TrajectoryFactory
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.drive.PointTurnCommand
import frc.robot.subsystems.drive.VisionDriveCommand
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.shooter.FlywheelSubsystem
import frc.robot.subsystems.shooter.ShootCommand
import lib.beforeStarting
import lib.instantCommand
import lib.runCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.inRadians
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.mathematics.units.seconds
import kotlin.math.absoluteValue

class OpposingTrenchRoutine : AutoRoutine() {
    private val path1 = TrajectoryFactory.eightPCAutoStartToOpposingTrench // Grab 2
    private val path2 = TrajectoryFactory.eightPCAutoOpposingTrenchToShoot // Post up next to truss
    private val path3 = TrajectoryFactory.retrieve5FromShieldGenerator
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

            +DriveSubsystem.followTrajectory(path2) { (160.0).degrees.toRotation2d() }

            val command = VisionDriveCommand()
            +command.withExit { command.lastError.absoluteValue < 1.5.degrees.inRadians() }
                    .deadlineWith(ShootCommand())

            +(FlywheelSubsystem.agitateAndShoot((3.seconds)))
                    .deadlineWith(VisionDriveCommand())

            val timer = Timer()
            +DriveSubsystem.followTrajectory2(path3) {
                // switch headings halfway through
                if(timer.get() < 2.5) 65.degrees else 115.degrees
            }
                    .beforeStarting { timer.reset(); timer.start() }
                    .deadlineWith(runCommand(IntakeSubsystem) { IntakeSubsystem.setSpeed(1.0) })
                    .andThen(Runnable { IntakeSubsystem.setNeutral() }, IntakeSubsystem)

            +DriveSubsystem.followTrajectory(path4) { 160.0.degrees.toRotation2d() }

            val command2 = VisionDriveCommand()
            +command2.withExit { command2.lastError.absoluteValue < 1.5.degrees.inRadians() }
                    .deadlineWith(ShootCommand())

            +(FlywheelSubsystem.agitateAndShoot((3.seconds)))
                    .deadlineWith(
                            VisionDriveCommand(),
                            runCommand(IntakeSubsystem) { IntakeSubsystem.setSpeed(1.0); IntakeSubsystem.setSmolPistonExtension(true) })
                    .andThen(Runnable { IntakeSubsystem.setNeutral() }, IntakeSubsystem)
        }
}
