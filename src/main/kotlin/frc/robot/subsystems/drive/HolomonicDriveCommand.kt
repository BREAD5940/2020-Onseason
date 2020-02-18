package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState
import frc.robot.Controls
import kotlin.math.absoluteValue
import lib.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.utils.withDeadband
import org.ghrobotics.lib.wrappers.hid.getX
import org.ghrobotics.lib.wrappers.hid.getY
import kotlin.properties.Delegates


open class HolomonicDriveCommand : FalconCommand(DriveSubsystem) {
    private var lastSpeed: ChassisSpeeds = ChassisSpeeds()
    private var wasEvading = false
    private var clockwiseCenter = Translation2d()
    private var counterClockwiseCenter = Translation2d()


    override fun execute() {
        var forward = -xSource() / 1.0
        var strafe = -zSource() / 1.0
        var rotation = -rotSource() * 1.0 / 1.0
       // var isRobotRelative = false
        forward *= forward.absoluteValue
        strafe *= strafe.absoluteValue
        rotation *= rotation.absoluteValue



        // calculate translation vector (with magnitude of the max speed
        // volts divided by volts per meter per second is meters per second
        val translation = Translation2d(forward, strafe) // this will have a norm of 1, or 100% power

        if (forward.absoluteValue < 0.01 && strafe.absoluteValue < 0.01 && rotation.absoluteValue < 0.01) {
            // Point wheels inwards
            DriveSubsystem.periodicIO.output = SwerveDriveOutput.KinematicsVoltage(
                    DriveSubsystem.kinematics.toSwerveModuleStates(
                            ChassisSpeeds(0.0, 0.0, 1.0)).map {
                        SwerveModuleState(0.0, it.angle + 90.degrees.toRotation2d())
                    }
            )
            return
        }

        // calculate wheel speeds from field oriented chassis state
        val speeds: ChassisSpeeds

        speeds = if (isRobotRelative()) {
//            println("in robot relative")
            ChassisSpeeds(translation.x, translation.y, rotation)
        } else {
//            println("field relative")
            ChassisSpeeds.fromFieldRelativeSpeeds(translation.x, translation.y, rotation, DriveSubsystem.periodicIO.pose.rotation)
        }

        DriveSubsystem.periodicIO.output = SwerveDriveOutput.Percent(speeds, Translation2d())

        this.lastSpeed = speeds
    }

     companion object {
        private val kTranslationHand = GenericHID.Hand.kRight
        private val kRotHand = GenericHID.Hand.kLeft
        val xSource by lazy { Controls.driverFalconXbox.getY(kTranslationHand).withDeadband(0.1) }
        val zSource by lazy { Controls.driverFalconXbox.getX(kTranslationHand).withDeadband(0.1) }
        val rotSource by lazy { Controls.driverFalconXbox.getX(kRotHand).withDeadband(0.06) }

         val isRobotRelative by lazy { Controls.driverFalconXbox.getRawButton(9) }
    }

}
