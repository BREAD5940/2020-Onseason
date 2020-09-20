package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import kotlin.math.absoluteValue

class PointTurnCommand(private val wantedAngle: Rotation2d) : VisionDriveCommand() {

    override fun execute() {

        val speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                0.0, 0.0, controller.calculate(DriveSubsystem.robotPosition.rotation.radians, wantedAngle.radians),
                DriveSubsystem.robotPosition.rotation)

        DriveSubsystem.periodicIO.output = SwerveDriveOutput.Percent(speeds, centerOfRotation)
    }

    override fun isFinished(): Boolean {
        return (DriveSubsystem.robotPosition.rotation.minus(wantedAngle).degrees.absoluteValue < 4)
    }

    override fun end(interrupted: Boolean) {
        DriveSubsystem.setNeutral()
    }
}
