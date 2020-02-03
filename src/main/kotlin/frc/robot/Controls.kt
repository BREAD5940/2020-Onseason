package frc.robot

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.auto.paths.TrajectoryFactory.testTrajectory
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.drive.SwerveCharacterizationCommand
import frc.robot.subsystems.drive.SwerveTrajectoryFollowerCommand
//import frc.robot.subsystems.drive.VisionDriveCommand
import frc.robot.subsystems.superstructure.intake.Intake
import frc.robot.subsystems.superstructure.shooter.Shooter
import frc.robot.subsystems.superstructure.slurp.SlurpBoi
import kotlinx.coroutines.Runnable
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.FalconTrajectoryConfig
import org.ghrobotics.lib.mathematics.twodim.trajectory.FalconTrajectoryGenerator
import org.ghrobotics.lib.mathematics.units.derived.*
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.wrappers.hid.* // ktlint-disable no-wildcard-imports

object Controls {

    val driverControllerLowLevel = XboxController(0)
    val driverFalconXbox = driverControllerLowLevel.mapControls {

        val rezeroCommand = { DriveSubsystem.setGyroAngle(0.degrees.toRotation2d()) }
        button(kBumperLeft).changeOn(rezeroCommand)
        button(kStart).changeOn(rezeroCommand)

    }

    fun update() {
        driverFalconXbox.update()
    }
}

//private fun Command.andThen(block: () -> Unit) = sequential { +this@andThen ; +InstantCommand(Runnable(block)) }
