package frc.robot

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.auto.paths.TrajectoryFactory.testTrajectory
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.drive.SwerveCharacterizationCommand
import frc.robot.subsystems.drive.SwerveTrajectoryFollowerCommand
import frc.robot.subsystems.drive.VisionDriveCommand
import frc.robot.subsystems.superstructure.ClosedLoopProximalMove
import frc.robot.subsystems.superstructure.Proximal
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
//        button(kX).changeOn(InstantCommand(Runnable { println("Hullo!") }))

        val rezeroCommand = { DriveSubsystem.setGyroAngle(0.degrees.toRotation2d()) }
        button(kBack).changeOn(rezeroCommand)
        button(kStart).changeOn(rezeroCommand)

//        val trajectory = FalconTrajectoryGenerator.generateTrajectory(
//                listOf(Pose2d(10.feet, 10.feet, 0.degrees), Pose2d( 20.feet, 5.feet, 0.degrees)),
//                FalconTrajectoryConfig(4.feet.velocity, 5.feet.acceleration)
//        )
        val command = SwerveTrajectoryFollowerCommand(testTrajectory, 180.degrees.toRotation2d())
        button(kA).changeOn(command)
        button(kX).changeOn(VisionDriveCommand())
//
//        button(kA).changeOn(SwerveCharacterizationCommand())
        button(kB).changeOn {
            DriveSubsystem.odometry.resetPosition(Pose2d(1.5.feet, 23.feet, 0.degree), DriveSubsystem.gyro())
        }

        button(kA).changeOn { Proximal.master.encoder.resetPosition(0.degrees) }
        pov(0).changeOn(ClosedLoopProximalMove(90.degrees))
        pov(90).changeOn(ClosedLoopProximalMove(0.degrees))

    }

//    val operatorWPIJoystick = XboxController(1)
//    val operatorFalconXbox = operatorWPIJoystick.mapControls {
//    }

    fun update() {
        driverFalconXbox.update()
//        operatorFalconHID.update()
//        operatorFalconXbox.update()
    }
}

private fun Command.andThen(block: () -> Unit) = sequential { +this@andThen ; +InstantCommand(Runnable(block)) }
