package frc.robot.subsystems.drive

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import org.ghrobotics.lib.commands.FalconCommand

class SwerveCharacterizationCommand : FalconCommand(DriveSubsystem) {

    private val numberArray = DoubleArray(9)

    private val autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed")
    private val telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry")

    override fun runsWhenDisabled() = true

    private var priorAutoSpeed = 0.0

    override fun execute() {
        val autospeed = autoSpeedEntry.getDouble(0.0)
        priorAutoSpeed = autospeed

        DriveSubsystem.periodicIO.output = SwerveDriveOutput.Percent(
                ChassisSpeeds(autospeed, 0.0, 0.0)
        )

        numberArray[0] = Timer.getFPGATimestamp()
        numberArray[1] = RobotController.getBatteryVoltage()
        numberArray[2] = autospeed
        numberArray[3] = DriveSubsystem.flModule.driveMotor.voltageOutput.value
        numberArray[4] = DriveSubsystem.frModule.driveMotor.voltageOutput.value
        numberArray[5] = DriveSubsystem.flModule.driveMotor.encoder.position.value
        numberArray[6] = DriveSubsystem.frModule.driveMotor.encoder.position.value
        numberArray[7] = DriveSubsystem.flModule.driveMotor.encoder.velocity.value
        numberArray[8] = DriveSubsystem.frModule.driveMotor.encoder.velocity.value

        telemetryEntry.setNumberArray(numberArray.toTypedArray())
    }

    override fun end(interrupted: Boolean) {
        DriveSubsystem.setNeutral()
    }
}
