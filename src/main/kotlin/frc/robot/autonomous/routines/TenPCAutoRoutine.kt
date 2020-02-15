package frc.robot.autonomous.routines

import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.auto.paths.TrajectoryFactory
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.intake.IntakeSubsystem
import lib.runCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d

class TenPCAutoRoutine: AutoRoutine() {
    private val path1 = TrajectoryFactory.tenPointAutoToShieldGenerator
    private val path2 = TrajectoryFactory.tenPointAutoShieldGeneratorToShoot
    private val path3 = TrajectoryFactory.tenPointAutoPCFromTrench
    private val path4 = TrajectoryFactory.tenPointAutoTrenchToShoot

    override val duration: SIUnit<Second>
        get() = path1.duration +
                path2.duration +
                path3.duration +
                path4.duration

    override val routine
        get() = sequential {
            +DriveSubsystem.followTrajectory2(path1) { (-70).degrees }
                    .alongWith(
                            IntakeSubsystem.extendIntakeCommand()
                                    .andThen(runCommand(IntakeSubsystem) { IntakeSubsystem.setSpeed(0.5) }))
                    .andThen(Runnable { IntakeSubsystem.setNeutral() }, IntakeSubsystem),
            +DriveSubsystem.followTrajectory(path2) { 0.0.degrees }
                    .andThen()




//            +DriveSubsystem.followTrajectory2(path2) {
//                if(Rectangle2d(Translation2d(0.0, 0.0), Translation2d(2.0,2.0))
//                                .contains(DriveSubsystem.robotPosition.translation))
//                    45.degrees else 50.degrees
//            }
        }


}