package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.geometry.Rotation2d
import frc.robot.auto.paths.TrajectoryFactory
import frc.robot.subsystems.vision.VisionSubsystem
import org.ghrobotics.lib.commands.FalconCommand

val trajectory = TrajectoryFactory.testTrajectory2
fun getEndHeading(): Rotation2d {
    if (DriveSubsystem.robotPosition.translation.getDistance(trajectory.states.last().poseMeters.translation) < 1.0) {
        return DriveSubsystem.robotPosition.rotation.plus(Rotation2d.fromDegrees(VisionSubsystem.xOffset))
    } else {
        return trajectory.states.last().poseMeters.rotation
    }
}

class VisionDriveCommand : FalconCommand() {
    override fun execute() {
        DriveSubsystem.followTrajectory(trajectory) { getEndHeading() }
    }
}
