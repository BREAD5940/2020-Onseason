package frc.robot

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.geometry.Pose2d
import frc.robot.auto.paths.TrajectoryFactory
import frc.robot.subsystems.climb.GrabBumperCommand
import frc.robot.subsystems.climb.openLoopClimbCommandGroup
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.drive.VisionDriveCommand
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.shooter.FlywheelSubsystem
import frc.robot.subsystems.shooter.ShootCommand
import lib.beforeStarting
import lib.instantCommand
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.wrappers.hid.*

object Controls {

    val kGutSpeed = 0.65 // 0.65 works well

    var isClimbing = false
    val driverWpiXbox = XboxController(0)
    val driverFalconXbox = driverWpiXbox.mapControls {
        val reZeroCommand = { DriveSubsystem.setGyroAngle(180.degrees.toRotation2d()) }

        button(kStart).changeOn(reZeroCommand)
        button(kA).changeOn(IntakeSubsystem.extendIntakeCommand())
        button(kX).changeOn { IntakeSubsystem.miniRetractIntakeCommand() }
        // button(kBumperRight).change(ShootCommand().alongWith(VisionDriveCommand()))
        button(kBumperRight).change(
                ShootCommand(true)
                        .andThen(ShootCommand(false).withTimeout(0.5))
                        .andThen(
                                ShootCommand(false)
                                        .beforeStarting(Runnable { FlywheelSubsystem.kickWheelMotor.setDutyCycle(kGutSpeed) })
                                        .andThen(Runnable { FlywheelSubsystem.kickWheelMotor.setNeutral() })
                        ).alongWith(VisionDriveCommand())
        )
        button(kBumperLeft).changeOn { FlywheelSubsystem.kickWheelMotor.setDutyCycle(kGutSpeed) }.changeOff { FlywheelSubsystem.kickWheelMotor.setNeutral() }
        button(kStickRight).change(ShootCommand({ Constants.rightBelowGoalParameter5v }))
        button(kY).change(ShootCommand().alongWith(VisionDriveCommand()))

//        pov(180).changeOn(
//                DriveSubsystem.followTrajectory(
//                                TrajectoryFactory.testTrajectory2
//                        ) { 0.degrees.toRotation2d() }
//                        .beforeStarting {
//                            DriveSubsystem.odometry.resetPosition(
//                                    Pose2d(TrajectoryFactory.testTrajectory2.initialPose.translation, 180.degrees.toRotation2d()), DriveSubsystem.gyro())
//                        }
//        )

        val timer = Timer()
        pov(180).changeOn(DriveSubsystem.followTrajectory2(TrajectoryFactory.retrieve5FromShieldGenerator) {
                    // switch headings halfway through
                    if (timer.get() < 2.5) 65.degrees else -68.degrees
                }
                .beforeStarting { timer.reset(); timer.start() }
//                .deadlineWith(runCommand(IntakeSubsystem) { IntakeSubsystem.setSpeed(1.0) })
//                .andThen(Runnable { IntakeSubsystem.setNeutral() }, IntakeSubsystem)
                .beforeStarting {
            DriveSubsystem.odometry.resetPosition(
                    Pose2d(TrajectoryFactory.retrieve5FromShieldGenerator.initialPose.translation, 180.degrees.toRotation2d()), DriveSubsystem.robotHeadingSource()) })

//        pov(0).changeOn(ShooterCharacterizationCommand())
//        pov(0).changeOn(SwerveCharacterizationCommand())
    }

    val operatorXbox = XboxController(1)
    val operatorFalconXbox = operatorXbox.mapControls {
        button(kBumperRight).change(
                ShootCommand(true)
                        .andThen(ShootCommand(false).withTimeout(0.5))
                        .andThen(
                                ShootCommand(false)
                                        .beforeStarting(Runnable { FlywheelSubsystem.kickWheelMotor.setDutyCycle(kGutSpeed) })
                                        .andThen(Runnable { FlywheelSubsystem.kickWheelMotor.setNeutral() })
                        ).alongWith(VisionDriveCommand())
        )

        button(kA).change(ShootCommand().alongWith(VisionDriveCommand())).changeOff { FlywheelSubsystem.kickWheelMotor.setNeutral(); FlywheelSubsystem.shooterMaster.setNeutral() }
        button(kX).changeOn(IntakeSubsystem.extendIntakeCommand())
        button(kY).change(IntakeSubsystem.retractIntakeCommand())
        pov(0).changeOn { openLoopClimbCommandGroup.alongWith(GrabBumperCommand(), instantCommand(IntakeSubsystem) {}).withInterrupt { operatorXbox.bButton }.schedule(false) }
        button(kStickRight).change(
                ShootCommand({ Constants.rightBelowGoalParameter5v }, true)
                        .andThen(ShootCommand({ Constants.rightBelowGoalParameter5v }, false).withTimeout(1.0))
                        .andThen(
                                ShootCommand({ Constants.rightBelowGoalParameter5v }, false)
                                        .beforeStarting(Runnable { FlywheelSubsystem.kickWheelMotor.setDutyCycle(1.0) })
                                        .andThen(Runnable { FlywheelSubsystem.kickWheelMotor.setNeutral() })
                        )
        )

        // TODO decide on button bindings for offset of hood
//        button(kStickLeft).changeOn { ShootCommand.hoodAngleOffset += 1.degrees }
//        button(kStickLeft).changeOn { ShootCommand.hoodAngleOffset -= 1.degrees }
    }

    fun update() {
        driverFalconXbox.update()
        operatorFalconXbox.update()
    }
}
