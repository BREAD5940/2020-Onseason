package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Transform2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.trajectory.Trajectory
import frc.robot.Constants
import frc.robot.auto.paths.TrajectoryFactory
import frc.robot.autonomous.paths.TrajectoryWaypoints
import frc.robot.autonomous.paths.transformBy
import frc.robot.subsystems.vision.VisionSubsystem
import kotlin.math.PI
import kotlin.math.absoluteValue
import lib.normalize
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.mathematics.units.kFeetToMeter

val trajectory = TrajectoryFactory.grabThreeAndShoot
val newPos = DriveSubsystem.robotPosition
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
