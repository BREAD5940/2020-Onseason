package frc.robot.autonomous.routines

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import frc.robot.auto.paths.TrajectoryFactory
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.drive.SwerveDriveOutput
import frc.robot.subsystems.drive.VisionDriveCommand
import frc.robot.subsystems.shooter.FlywheelSubsystem
import lib.instantCommand
import lib.runCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.mathematics.units.seconds

class ThreePCRoutine : AutoRoutine() {
    private val path1 = TrajectoryFactory.moveForward5Feet

    override val duration: SIUnit<Second>
        get() = SIUnit<Second>(path1.totalTimeSeconds)
    override val routine
        get() = sequential {
            +instantCommand { DriveSubsystem.robotPosition = Pose2d(path1.states.first().poseMeters.translation, 0.degrees.toRotation2d()) }

            +runCommand(DriveSubsystem) {
                DriveSubsystem.periodicIO.output = SwerveDriveOutput.Percent(ChassisSpeeds(-.2, 0.0, 0.0), Translation2d())
            }.withTimeout(1.0)

            +instantCommand(DriveSubsystem) { DriveSubsystem.setNeutral() }

            +(FlywheelSubsystem.agitateAndShoot((4.seconds)))
                    .deadlineWith(VisionDriveCommand())
        }
}
