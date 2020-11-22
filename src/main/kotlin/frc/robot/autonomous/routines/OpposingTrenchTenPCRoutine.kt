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
import kotlin.math.absoluteValue
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

class OpposingTrenchTenPCRoutine : AutoRoutine() {
    private val path1 = TrajectoryFactory.tenPCAutoStartToOpposingTrench // Grab 2 from opposing trench
    private val path2 = TrajectoryFactory.tenPCAutoOpposingTrenchToShoot // Post up next to truss
    private val path3 = TrajectoryFactory.retrieve3FromShieldGenerator1 //Grab 2 of 3 from one side of truss
    private val path4 = TrajectoryFactory.retrieve3FromShieldGenerator2 //Grab last of 3 from one side of truss
    private val path5 = TrajectoryFactory.goAroundCornerOfShieldGenerator //Go around corner of shield generator
    private val path6 = TrajectoryFactory.retrieve2FromShieldGenerator //Grab 2 of 2 from one side of truss
    private val path7 = TrajectoryFactory.tenPCShieldGeneratorToShoot //Go to shoot

    private val command = VisionDriveCommand()

    override val duration: SIUnit<Second>
        get() = SIUnit(path1.totalTimeSeconds + path2.totalTimeSeconds + path3.totalTimeSeconds + path4.totalTimeSeconds + path5.totalTimeSeconds + path6.totalTimeSeconds + path7.totalTimeSeconds)

    override val routine
        get() = sequential {
            +instantCommand { DriveSubsystem.robotPosition = Pose2d(path1.states.first().poseMeters.translation, 0.degrees.toRotation2d()) }

            +DriveSubsystem.followTrajectory(path1) { 0.degrees.toRotation2d() }
                    .deadlineWith(
                            IntakeSubsystem.extendIntakeCommand()
                                    .andThen(runCommand(IntakeSubsystem) { IntakeSubsystem.setSpeed(1.0) }))

            +PointTurnCommand(180.degrees.toRotation2d())

            +DriveSubsystem.followTrajectory(path2) { 171.degrees.toRotation2d() }

            +(FlywheelSubsystem.agitateAndShoot((2.seconds)))
                    .deadlineWith(VisionDriveCommand())
                    .withExit { command.lastError.absoluteValue < 1.5.degrees.inRadians() }

            +PointTurnCommand(24.degrees.toRotation2d())

            //val timer = Timer()
            +DriveSubsystem.followTrajectory(path3)  { 67.degrees.toRotation2d() } //TODO Make sure it doesn't hit boundaries (bumps) under shield generator
                    //.beforeStarting { timer.reset(); timer.start() }
                    .deadlineWith(runCommand(IntakeSubsystem) { IntakeSubsystem.setSpeed(1.0) })
                    .andThen(Runnable { IntakeSubsystem.setNeutral() }, IntakeSubsystem)

            +DriveSubsystem.followTrajectory(path4) { -68.degrees.toRotation2d() }

            val command2 = VisionDriveCommand()
            +command2.withExit { command2.lastError.absoluteValue < 1.5.degrees.inRadians() }
                    .deadlineWith(ShootCommand())

            +(FlywheelSubsystem.agitateAndShoot((2.seconds)))
                    .deadlineWith(
                            VisionDriveCommand())
                            runCommand(IntakeSubsystem) { IntakeSubsystem.setSpeed(1.0); IntakeSubsystem.setSmolPistonExtension(true) })
                    .andThen(Runnable { IntakeSubsystem.setNeutral() }, IntakeSubsystem)

            println(duration)
        }
}
