package frc.robot.autonomous.routines

import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.auto.paths.TrajectoryFactory
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.shooter.FlywheelSubsystem
import lib.revolutionsPerMinute
import lib.runCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.mathematics.units.derived.velocity
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.mathematics.units.seconds

class MoveOffStart: AutoRoutine() {
    private val path1 = TrajectoryFactory.moveForward5Feet

    override val duration: SIUnit<Second>
        get() = path1.duration

    override val routine
        get() = sequential {
            +DriveSubsystem.followTrajectory2(path1) { 0.0.degrees }
//            +DriveSubsystem.followTrajectory2(path2) {
//                if(Rectangle2d(Translation2d(0.0, 0.0), Translation2d(2.0,2.0))
//                                .contains(DriveSubsystem.robotPosition.translation))
//                    45.degrees else 50.degrees
//            }
        }
}