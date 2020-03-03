package frc.robot.autonomous.routines

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.auto.paths.TrajectoryFactory
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.drive.PointTurnCommand
import frc.robot.subsystems.drive.VisionDriveCommand
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.shooter.FlywheelSubsystem
import frc.robot.subsystems.shooter.ShootCommand
import lib.instantCommand
import lib.runCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.inRadians
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.seconds
import kotlin.math.absoluteValue

class SixPCFromTrenchRoutine : AutoRoutine() {
    private val path1 = TrajectoryFactory.sixPCStartToShoot
    private val path2 = TrajectoryFactory.sixPCGrab3FromTrench
    private val path3 = TrajectoryFactory.sixPCReturnFromTrench

    override val duration: SIUnit<Second>
        get() = SIUnit(path1.totalTimeSeconds + path2.totalTimeSeconds + path3.totalTimeSeconds)

    override val routine
        get() = sequential {
            +instantCommand { DriveSubsystem.robotPosition = Pose2d(path1.states.first().poseMeters.translation, 180.degrees.toRotation2d()) }

            +DriveSubsystem.followTrajectory(path1) { -170.0.degrees.toRotation2d() }
                    .alongWith(IntakeSubsystem.extendIntakeCommand())
                    .deadlineWith(instantCommand { IntakeSubsystem.intakeMotor.setDutyCycle(0.5); IntakeSubsystem.setSmolPistonExtension(true) })

            val command = VisionDriveCommand()
            +command.withExit { command.lastError.absoluteValue < 1.5.degrees.inRadians() }
                    .deadlineWith(ShootCommand())

            +(FlywheelSubsystem.agitateAndShoot(2.5.seconds))
                    .deadlineWith(VisionDriveCommand())
                    .deadlineWith(instantCommand { IntakeSubsystem.intakeMotor.setDutyCycle(0.5); IntakeSubsystem.setSmolPistonExtension(true) })

//            +PointTurnCommand(22.degrees.toRotation2d())

            +DriveSubsystem.followTrajectory2(path2) { (0.0).degrees }
                    .deadlineWith(
                            IntakeSubsystem.extendIntakeCommand()
                                    .andThen(runCommand(IntakeSubsystem) { IntakeSubsystem.setSpeed(1.0) }))
                    .andThen(Runnable { IntakeSubsystem.setNeutral() }, IntakeSubsystem)
                    .alongWith(instantCommand { FlywheelSubsystem.kickWheelMotor.setDutyCycle(-0.5) }.andThen(Runnable{ FlywheelSubsystem.kickWheelMotor.setNeutral() }))

//            +PointTurnCommand(180.degrees.toRotation2d())

            val rectangle = Rectangle2d(Translation2d(27.feet, 22.feet), Translation2d(36.feet, 27.feet))

            +DriveSubsystem.followTrajectory2(path3) {
                if(rectangle.contains(DriveSubsystem.robotPosition.translation)) 0.degrees
                else (-172).degrees
            }
                    .deadlineWith(ShootCommand())
//                    .alongWith(
//                            IntakeSubsystem.retractIntakeCommand().andThen(WaitCommand(0.5))
//                                    .andThen(IntakeSubsystem.extendIntakeCommand())
//                    )
                    .deadlineWith(instantCommand { IntakeSubsystem.intakeMotor.setDutyCycle(0.5); IntakeSubsystem.setSmolPistonExtension(true) })


            // re-align with target
            val command2 = VisionDriveCommand()
            +command2.withExit { command2.lastError.absoluteValue < 1.5.degrees.inRadians() }
                    .deadlineWith(ShootCommand())

            +(FlywheelSubsystem.agitateAndShoot(3.seconds))
                    .deadlineWith(
                            VisionDriveCommand(),
                            runCommand(IntakeSubsystem) { IntakeSubsystem.setSpeed(1.0); IntakeSubsystem.setSmolPistonExtension(true) })
                    .andThen(Runnable { IntakeSubsystem.setNeutral() }, IntakeSubsystem)
                    .deadlineWith(instantCommand { IntakeSubsystem.intakeMotor.setDutyCycle(0.5); IntakeSubsystem.setSmolPistonExtension(true) })
        }
}
