package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Translation2d
import lib.Logger
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.inAmps
import org.ghrobotics.lib.mathematics.units.inMeters

class DataCollectionCommand : FalconCommand() {
    private val logger = Logger("DataCollection")

    override fun runsWhenDisabled(): Boolean {
        return false;
    }

    override fun execute() {
        // One list for each module
        var stateString = "${Timer.getFPGATimestamp()},"
        DriveSubsystem.modules.forEach {
            stateString += listOf(
                it.azimuthAngle().radians,
                it.azimuthMotor.encoder.position.value,
                it.azimuthMotor.encoder.velocity.value,
                it.azimuthMotor.voltageOutput.value,
                it.azimuthMotor.drawnCurrent.inAmps(),

                it.driveMotor.encoder.position.inMeters(),
                it.driveMotor.encoder.velocity.value,
                it.driveMotor.voltageOutput.value,
                it.driveMotor.drawnCurrent.inAmps()
            ).joinToString()
        }
        stateString += ",${DriveSubsystem.robotHeadingSource().radians}"
        val out = DriveSubsystem.periodicIO.output
        print(out)
        if (out is SwerveDriveOutput.KinematicsVoltage) {
            stateString += ("," + (out.speeds.map { it.angle.radians } + out.speeds.map { it.speedMetersPerSecond }).joinToString())
        }
        println(stateString)
        logger.log(stateString)
    }

}
