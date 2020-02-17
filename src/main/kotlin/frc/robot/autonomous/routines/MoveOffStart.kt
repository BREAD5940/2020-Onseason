package frc.robot.autonomous.routines

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.auto.paths.TrajectoryFactory
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.drive.SwerveDriveOutput
import frc.robot.subsystems.drive.VisionDriveCommand
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.shooter.FlywheelSubsystem
import frc.robot.subsystems.shooter.ShootCommand
import lib.instantCommand
import lib.runCommand
import org.ghrobotics.lib.commands.parallelRace
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.mathematics.units.seconds

class MoveOffStart : AutoRoutine() {

    override val duration: SIUnit<Second>
        get() = 0.seconds

    override val routine
        get() = sequential {
            +instantCommand(DriveSubsystem) { DriveSubsystem.setGyroAngle(0.degrees.toRotation2d()) }
            +runCommand(DriveSubsystem) {
                DriveSubsystem.periodicIO.output = SwerveDriveOutput.Percent(
                        ChassisSpeeds(-0.2, 0.0, 0.0)) }
                    .withTimeout(1.0)

            +parallelRace {
                +VisionDriveCommand()
                +ShootCommand()
                +sequential {
                    +WaitCommand(2.0)
                    +runCommand(IntakeSubsystem) { FlywheelSubsystem.kickWheelMotor.setDutyCycle(.8) }
                            .withTimeout(5.0)
                }
            }

            +IntakeSubsystem.extendIntakeCommand()

        }
}
