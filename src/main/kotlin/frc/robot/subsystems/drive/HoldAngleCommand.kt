package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import kotlin.math.absoluteValue

class HoldAngleCommand : VisionDriveCommand() {

    lateinit var wantedAngle: Rotation2d

    override fun initialize() {
        wantedAngle = DriveSubsystem.robotPosition.rotation
    }

    override fun execute() {

        var forward = -xSource() / 1.0
        var strafe = -zSource() / 1.0
        forward *= forward.absoluteValue
        strafe *= strafe.absoluteValue

        val speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                forward, strafe, controller.calculate(DriveSubsystem.robotPosition.rotation.radians, wantedAngle.radians),
                DriveSubsystem.robotPosition.rotation)

        DriveSubsystem.periodicIO.output = SwerveDriveOutput.Percent(speeds, centerOfRotation)
    }
}
