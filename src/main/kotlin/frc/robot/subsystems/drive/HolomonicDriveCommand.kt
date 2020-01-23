package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import frc.robot.Constants
import frc.robot.Controls
import kotlin.math.absoluteValue
import lib.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.utils.withDeadband
import org.ghrobotics.lib.wrappers.hid.getX
import org.ghrobotics.lib.wrappers.hid.getY

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

        // calculate translation vector (with magnitude of the max speed
        // volts divided by volts per meter per second is meters per second
        val translation = Translation2d(forward, strafe) // this will have a norm of 1, or 100% power

        if (forward.absoluteValue < 0.01 && strafe.absoluteValue < 0.01 && rotation.absoluteValue < 0.01) {
            DriveSubsystem.periodicIO.output = SwerveDriveOutput.Nothing
//            val states = DriveSubsystem.currentSwerveModuleStates
//            DriveSubsystem.periodicIO.output = SwerveDriveOutput.KinematicsVelocity()
            return
        }

        // calculate wheel speeds from field oriented chassis state
        val speeds: ChassisSpeeds
        if (!isRobotRelative()) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.x, translation.y, rotation, DriveSubsystem.periodicIO.pose.rotation)
        } else {
            speeds = ChassisSpeeds(translation.x, translation.y, rotation)
        }
//        val moduleStates = DriveSubsystem.kinematics.toSwerveModuleStates(speeds, centerOfRotation)

        // Normalize wheel speeds
        // Max duty cycle is 1.0
//        SwerveDriveKinematics.normalizeWheelSpeeds(speeds,1.0)

//        println("speeds ${speeds.toString2()}")

        DriveSubsystem.periodicIO.output = SwerveDriveOutput.Percent(speeds)

        this.lastSpeed = speeds
    }

    /** Determine which wheels to use to evade. */
    private fun determineEvasionWheels(driveVector: Translation2d, robotPosition: Pose2d) {
        val here: Translation2d = driveVector.rotateBy(robotPosition.rotation.inverse())
        val wheels = Constants.kModulePositions
        clockwiseCenter = wheels[0]
        counterClockwiseCenter = wheels[wheels.size - 1]
        for (i in 0 until wheels.size - 1) {
            val cw = wheels[i]
            val ccw = wheels[i + 1]
            if (here.isWithinAngle(cw, ccw)) {
                clockwiseCenter = ccw
                counterClockwiseCenter = cw
            }
        }
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
