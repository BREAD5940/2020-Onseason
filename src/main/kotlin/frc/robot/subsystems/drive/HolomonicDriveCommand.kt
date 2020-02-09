package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState
import frc.robot.Constants
import frc.robot.Controls
import kotlin.math.absoluteValue
import lib.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.utils.withDeadband
import org.ghrobotics.lib.wrappers.hid.getX
import org.ghrobotics.lib.wrappers.hid.getY
import kotlin.math.abs

class HolomonicDriveCommand : FalconCommand(DriveSubsystem) {

    private var lastSpeed: ChassisSpeeds = ChassisSpeeds()
    private var wasEvading = false
    private var clockwiseCenter = Translation2d()
    private var counterClockwiseCenter = Translation2d()

    override fun execute() {
        var forward = -xSource() / 1.0
        var strafe = -zSource() / 1.0
        var rotation = -rotSource() * 1.0 / 1.0

        forward *= forward.absoluteValue
        strafe *= strafe.absoluteValue
        rotation *= rotation.absoluteValue

        // if abs of forward, strafe and rotation are all less than 0.01, do this and return
        if (abs(forward) < 0.01 && abs(strafe) < 0.01 && abs(rotation) < 0.01) { //should be false (I think?)
            DriveSubsystem.periodicIO.output = SwerveDriveOutput.KinematicsVoltage(pointInwards())
            return
        }

        // calculate translation vector (with magnitude of the max speed
        // volts divided by volts per meter per second is meters per second
        val translation = Translation2d(forward, strafe) // this will have a norm of 1, or 100% power

        if (forward.absoluteValue < 0.01 && strafe.absoluteValue < 0.01 && rotation.absoluteValue < 0.01) {
            DriveSubsystem.periodicIO.output = SwerveDriveOutput.Nothing
            return
        }

        // calculate wheel speeds from field oriented chassis state
        val speeds: ChassisSpeeds
        if (!isRobotRelative()) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.x, translation.y, rotation, DriveSubsystem.periodicIO.pose.rotation)
        } else {
            speeds = ChassisSpeeds(translation.x, translation.y, rotation)
        }

        DriveSubsystem.periodicIO.output = SwerveDriveOutput.Percent(speeds, Translation2d(.50, 0.0))

        this.lastSpeed = speeds
    }

    private fun pointInwards(): List<SwerveModuleState> {
        // what we'll do it just rotate, then rotate the angles by 90 degrees
        // and set the speeds to 0

        return Constants.kinematics.toSwerveModuleStates(ChassisSpeeds(
                0.0, 0.0, 1.0))
                .map { it -> SwerveModuleState(0.0, it.angle + 90.degrees.toRotation2d()) }

    }

    companion object {
        private val kTranslationHand = GenericHID.Hand.kRight
        private val kRotHand = GenericHID.Hand.kLeft
        val xSource by lazy { Controls.driverFalconXbox.getY(kTranslationHand).withDeadband(0.1) }
        val zSource by lazy { Controls.driverFalconXbox.getX(kTranslationHand).withDeadband(0.1) }
        val rotSource by lazy { Controls.driverFalconXbox.getX(kRotHand).withDeadband(0.06) }
        val isRobotRelative by lazy { Controls.driverFalconXbox.getRawButton(11) } // TODO check

        var centerOfRotation = Translation2d()
    }
}

private fun ChassisSpeeds.toString2(): String {
    return "Speeds vx $vxMetersPerSecond vy $vyMetersPerSecond omega $omegaRadiansPerSecond"
}
