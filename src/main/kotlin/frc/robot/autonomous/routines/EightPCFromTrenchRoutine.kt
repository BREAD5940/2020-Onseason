package frc.robot.autonomous.routines

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Pose2d
import frc.robot.auto.paths.TrajectoryFactory
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.drive.PointTurnCommand
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

class EightPCFromTrenchRoutine : AutoRoutine() {
    private val path1 = TrajectoryFactory.shootThree // sets up to shoot starting balls
    private val path2 = TrajectoryFactory.getPCFromTrench // gets 5 balls in trench
    private val path3 = TrajectoryFactory.trenchToShoot // goes back to shooting spot

    override val duration: SIUnit<Second>
        get() = SIUnit<Second>(path1.totalTimeSeconds + path2.totalTimeSeconds + path3.totalTimeSeconds)

    override val routine
        get() = sequential {

            +instantCommand { DriveSubsystem.robotPosition = Pose2d(path1.states.first().poseMeters.translation, 180.degrees.toRotation2d()) }

            +DriveSubsystem.followTrajectory(path1, { -166.degrees.toRotation2d() })
                    .alongWith(IntakeSubsystem.extendIntakeCommand())

            +(FlywheelSubsystem.agitateAndShoot(2.seconds))
                    .deadlineWith(VisionDriveCommand())

            +PointTurnCommand(0.degrees.toRotation2d())

            +DriveSubsystem.followTrajectory2(path2) { 0.degrees }
                    .deadlineWith(runCommand(IntakeSubsystem) { IntakeSubsystem.setSpeed(1.0) })
                    .andThen(Runnable { IntakeSubsystem.setNeutral() }, IntakeSubsystem)

            +PointTurnCommand(10.degrees.toRotation2d())

            +PointTurnCommand(-10.degrees.toRotation2d())

            val timer = Timer()
            +DriveSubsystem.followTrajectory(path3) { if (timer.get() > 1.0) 180.degrees.toRotation2d() else 0.degrees.toRotation2d() }
                    .beforeStarting { timer.reset(); timer.start() }

            +(FlywheelSubsystem.agitateAndShoot(2.seconds))
                    .deadlineWith(VisionDriveCommand(),
                        runCommand(IntakeSubsystem) { IntakeSubsystem.setSpeed(1.0); IntakeSubsystem.setSmolPistonExtension(true) })
                    .andThen(Runnable { IntakeSubsystem.setNeutral() }, IntakeSubsystem)
        }
}
