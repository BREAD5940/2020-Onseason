package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.auto.paths.TrajectoryFactory
import frc.robot.subsystems.vision.VisionSubsystem
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.meters
import kotlin.math.absoluteValue


class VisionDriveCommand : HolomonicDriveCommand() {

    init {
        SmartDashboard.putData("vision PID", controller)
    }

    override fun execute() {
        var forward = -xSource() / 1.0
        var strafe = -zSource() / 1.0
        forward *= forward.absoluteValue
        strafe *= strafe.absoluteValue

        when {
            VisionSubsystem.ps3eye.isValid -> {
                val speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        forward, strafe, controller.calculate(VisionSubsystem.ps3eye.yaw.radians, 0.0),
                        DriveSubsystem.robotPosition.rotation)

                DriveSubsystem.periodicIO.output = SwerveDriveOutput.Percent(speeds, centerOfRotation)
            }
            VisionSubsystem.lifecam.isValid -> {

                val speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        forward, strafe, controller.calculate(VisionSubsystem.lifecam.yaw.radians, 0.0),
                        DriveSubsystem.robotPosition.rotation)

                DriveSubsystem.periodicIO.output = SwerveDriveOutput.Percent(speeds, centerOfRotation)
            }
            else -> {
                super.execute()
            }
        }
    }

    companion object {
        val centerOfRotation = Translation2d(0.meters, 8.inches)
        val controller = PIDController(1.0, 0.0, 0.0)
    }
}
