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

class RightSideTenPCRoutine : AutoRoutine() {
    private val path1 = TrajectoryFactory.tenPCAutoToShieldGenerator
    private val path2 = TrajectoryFactory.tenPCAutoShieldGeneratorToShoot
    private val path3 = TrajectoryFactory.getPCFromTrench
    private val path4 = TrajectoryFactory.shootFromTrench

    override val duration: SIUnit<Second>
        get() = path1.duration +
                path2.duration +
                path3.duration +
                path4.duration

    override val routine
        get() = sequential {
            +instantCommand { DriveSubsystem.robotPosition = Pose2d(path1.states.first().poseMeters.translation, 0.degrees.toRotation2d()) }

            +DriveSubsystem.followTrajectory2(path1) { (-68).degrees }
                    .alongWith(
                            IntakeSubsystem.extendIntakeCommand()
                                    .andThen(runCommand(IntakeSubsystem) { IntakeSubsystem.setSpeed(1.0) }))//TODO wait a certen amount before turning intake off
                    .andThen(Runnable { IntakeSubsystem.setNeutral() }, IntakeSubsystem)

            +PointTurnCommand(-68.degrees.toRotation2d())

            +DriveSubsystem.followTrajectory(path2) { -166.degrees.toRotation2d() }

            +PointTurnCommand(-166.degrees.toRotation2d())

            +(FlywheelSubsystem.agitateAndShoot((3.5.seconds)))
                    .deadlineWith(VisionDriveCommand()) //TODO Try spinning up the shooter earlier, rather then now

            +PointTurnCommand(0.degrees.toRotation2d())

            +DriveSubsystem.followTrajectory(path3) { 0.0.degrees.toRotation2d() }
                    .alongWith(
                            IntakeSubsystem.extendIntakeCommand()
                                    .andThen(runCommand(IntakeSubsystem) { IntakeSubsystem.setSpeed(1.0) }))//TODO same here
                    .andThen(Runnable { IntakeSubsystem.setNeutral() }, IntakeSubsystem)

            +PointTurnCommand(180.degrees.toRotation2d())

            +DriveSubsystem.followTrajectory(path4) { 180.0.degrees.toRotation2d() }
                    //TODO Implement a "ammo" sensor so that we can shoot the right amount.
            +(FlywheelSubsystem.agitateAndShoot((3.5.seconds)))
                    .deadlineWith(VisionDriveCommand(),
                            runCommand(IntakeSubsystem) { IntakeSubsystem.setSpeed(1.0); IntakeSubsystem.setSmolPistonExtension(true) })
                    .andThen(Runnable { IntakeSubsystem.setNeutral() }, IntakeSubsystem)
        }
}
