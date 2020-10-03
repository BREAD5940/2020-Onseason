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

class OpposingTrenchTenRoutine : AutoRoutine() {
    private val path1 = TrajectoryFactory.tenPCAutoStartToOpposingTrench // Grab 2 from opposing trench
    private val path2 = TrajectoryFactory.tenPCAutoOpposingTrenchToShoot // Post up next to truss
    private val path3 = TrajectoryFactory.retrieve5FromShieldGenerator //Grab 5 from shield generator
    private val path4 = TrajectoryFactory.tenPCShieldGeneratorToShoot //Go to shoot

    private val command = VisionDriveCommand()

    override val duration: SIUnit<Second>
        get() = SIUnit(path1.totalTimeSeconds + path2.totalTimeSeconds + path3.totalTimeSeconds + path4.totalTimeSeconds)

    override val routine
        get() = sequential {
            +instantCommand { DriveSubsystem.robotPosition = Pose2d(path1.states.first().poseMeters.translation, 0.degrees.toRotation2d()) }

            +DriveSubsystem.followTrajectory2(path1) { 0.degrees }
                    .deadlineWith(
                            IntakeSubsystem.extendIntakeCommand()
                                    .andThen(runCommand(IntakeSubsystem) { IntakeSubsystem.setSpeed(1.0) }))

            +PointTurnCommand(180.degrees.toRotation2d())

            +DriveSubsystem.followTrajectory(path2) { 180.degrees.toRotation2d() }

            +(FlywheelSubsystem.agitateAndShoot((3.seconds)))
                    .deadlineWith(VisionDriveCommand())
                    //.withExit { command.lastError.absoluteValue < 1.5.degrees.inRadians() }

            +PointTurnCommand(67.degrees.toRotation2d())

            //val timer = Timer()
            +DriveSubsystem.followTrajectory(path3)  { 67.degrees.toRotation2d() }
                    //.beforeStarting { timer.reset(); timer.start() }
                    .deadlineWith(runCommand(IntakeSubsystem) { IntakeSubsystem.setSpeed(1.0) })
                    //.andThen(Runnable { IntakeSubsystem.setNeutral() }, IntakeSubsystem)

            +DriveSubsystem.followTrajectory(path4) { -112.degrees.toRotation2d() }

//            val command2 = VisionDriveCommand()
//            +command2.withExit { command2.lastError.absoluteValue < 1.5.degrees.inRadians() }
//                    .deadlineWith(ShootCommand())

            +(FlywheelSubsystem.agitateAndShoot((3.seconds)))
                    .deadlineWith(
                            VisionDriveCommand())
//                            runCommand(IntakeSubsystem) { IntakeSubsystem.setSpeed(1.0); IntakeSubsystem.setSmolPistonExtension(true) })
//                    .andThen(Runnable { IntakeSubsystem.setNeutral() }, IntakeSubsystem)
        }
}
