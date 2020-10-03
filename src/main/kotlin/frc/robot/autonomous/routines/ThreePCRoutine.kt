package frc.robot.autonomous.routines

import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.drive.SwerveDriveOutput
import frc.robot.subsystems.drive.VisionDriveCommand
import frc.robot.subsystems.shooter.FlywheelSubsystem
import frc.robot.subsystems.shooter.ShootCommand
import lib.instantCommand
import lib.runCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.mathematics.units.seconds

class ThreePCRoutine : AutoRoutine() {

    override val duration: SIUnit<Second>
        get() = 0.seconds

    override val routine
        get() = sequential {
            +runCommand(DriveSubsystem) {
                DriveSubsystem.periodicIO.output = SwerveDriveOutput.Percent(ChassisSpeeds(-.2, 0.0, 0.0), Translation2d())
            }.withTimeout(2.0)
                    .deadlineWith(ShootCommand())

            +instantCommand(DriveSubsystem) { DriveSubsystem.setNeutral() }

            +(FlywheelSubsystem.agitateAndShoot((4.seconds)))
                    .deadlineWith(VisionDriveCommand())
        }
}
