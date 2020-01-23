package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.geometry.Rotation2d
import frc.robot.auto.paths.TrajectoryFactory
import frc.robot.subsystems.vision.VisionSubsystem
import org.ghrobotics.lib.commands.FalconCommand

val trajectory = TrajectoryFactory.grabThreeAndShoot
fun getEndHeading(): Rotation2d {
    if(DriveSubsystem.robotPosition.minus(trajectory.states.last().poseMeters).translation.x < 1.0 && DriveSubsystem.robotPosition.minus(trajectory.states.last().poseMeters).translation.y < 1.0) {
        return DriveSubsystem.robotPosition.rotation.plus( Rotation2d.fromDegrees(VisionSubsystem.xOffset))
    } else {
        return trajectory.states.last().poseMeters.rotation
    }
}

class VisionDriveCommand : FalconCommand() {
    override fun execute() {
        DriveSubsystem.followTrajectory(trajectory, getEndHeading())
    }
}
