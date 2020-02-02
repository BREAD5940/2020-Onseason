package frc.test

import frc.test.SwerveEvadeTest.Constants.kModulePositions
import java.lang.Math.*
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import org.ghrobotics.lib.commands.FalconCommand

class SwerveStopTest : FalconCommand(DriveSubsystem) {
    fun something() {

        for (i in kModulePositions) {
            if (i.useState == SwerveDriveOutput.Nothing) {

            }
        }
    }
}