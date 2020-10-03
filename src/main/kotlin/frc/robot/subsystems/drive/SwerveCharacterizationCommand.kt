package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import lib.Logger
import org.ghrobotics.lib.commands.FalconCommand

class SwerveCharacterizationCommand : FalconCommand(DriveSubsystem) {

    override fun runsWhenDisabled() = false

    private var priorAutoSpeed = 0.0

    companion object {
        val logger = Logger("characterization")
    }

    override fun initialize() {
        logger.log("time, batt, percent, voltageOutput, velocity")
    }

    override fun execute() {
        priorAutoSpeed = 1.0
        println(priorAutoSpeed)

        DriveSubsystem.periodicIO.output = SwerveDriveOutput.Percent(
                ChassisSpeeds(priorAutoSpeed, 0.0, 0.0)
        )

        logger.log(Timer.getFPGATimestamp(), RobotController.getBatteryVoltage(), priorAutoSpeed, DriveSubsystem.flModule.driveMotor.voltageOutput.value, DriveSubsystem.flModule.driveMotor.encoder.velocity.value)
    }

    override fun end(interrupted: Boolean) {
        DriveSubsystem.setNeutral()
    }
}
