package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import frc.robot.Constants
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

class VisionDriveCommand : FalconCommand(DriveSubsystem) {

    lateinit var targetHeading: Rotation2d

    override fun runsWhenDisabled() = true

    private val rotationController = PIDController(10.0, 0.0, 0.0) // rad per sec per radian of error
            .apply {
                enableContinuousInput(-PI, PI)
            }
    private val translationController = PIDController(100.0, 0.0, 0.0)

    val rotationRange = -2 * PI..2 * PI
    val translationOutputRange = -4 * kFeetToMeter..4 * kFeetToMeter

    override fun initialize() {
        targetHeading = (importantAngles.minBy { (it - DriveSubsystem.robotPosition.rotation.radians).absoluteValue })!!.radians.toRotation2d()
        rotationController.setpoint = targetHeading.radians
        translationController.setpoint = 0.0
    }

    override fun execute() {
        val turn = rotationController.calculate(DriveSubsystem.robotPosition.rotation.radians)
                .coerceIn(rotationRange)

        val currentPose = DriveSubsystem.robotPosition
        // no clue if this works but here goes
        var error: Translation2d
        if (VisionSubsystem.getHasTargets()) {
            error = Translation2d(VisionSubsystem.getXOffset(), VisionSubsystem.getYOffset())
        } else {
            error = Translation2d(0.0, 0.0) // TODO: do something here for when the limelight can't see
        }
        println("current $currentPose error $error")
        val targetVelocity = translationController.calculate(error.norm, 0.020).coerceIn(translationOutputRange)
        error = error.normalize()
        val vX = -error.x * targetVelocity
        val vY = -error.y * targetVelocity

        DriveSubsystem.periodicIO.output = SwerveDriveOutput.Velocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(vX, vY, turn, DriveSubsystem.robotPosition.rotation)
        )
        println("Target Heading ${targetHeading.degrees} turn $turn")
    }

    companion object {

        val importantAngles = listOf(
                // rocket n, bay and f (mirrored and not)
                TrajectoryWaypoints.kSideStart.rotation /// TODO: have actual waypoints
        ).map { it.radians }
    }
}
