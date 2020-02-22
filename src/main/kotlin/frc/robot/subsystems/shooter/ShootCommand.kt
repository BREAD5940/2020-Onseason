package frc.robot.subsystems.shooter

import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Constants
import frc.robot.subsystems.drive.VisionDriveCommand
import frc.robot.subsystems.vision.VisionSubsystem
import kotlin.math.abs
import lib.inRpm
import lib.revolutionsPerMinute
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.Velocity
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.inDegrees

/**
 * Set the flywheel to shoot at a speed specified by a supplier. This command will run
 * forever if [endAfterSpinup] is false, and will exit when the hood and flywheel
 * are spun up and at the correct angle if [endAfterSpinup] is true.
 */
class ShootCommand(private val parameterSupplier: () -> ShotParameter, private val endAfterSpinup: Boolean = false) : FalconCommand(FlywheelSubsystem, HoodSubsystem) {

//    constructor(endAfterSpinup: Boolean = false) : this(
//            { FlywheelSubsystem.defaultShotLookupTable.get((VisionDriveCommand.getTargetPose() ?: Pose2d()).translation.norm) ?: ShotParameter.DefaultParameter }
//            , endAfterSpinup)

    constructor(endAfterSpinup: Boolean = false) : this(
            { Constants.pitchLookupTable5v.get((
                    if(VisionSubsystem.ps3eye.isValid) VisionSubsystem.ps3eye.pitch.degrees else 0.0))
                    ?: ShotParameter.DefaultParameter }, endAfterSpinup)

    constructor(hoodAngle: SIUnit<Radian>, speed: SIUnit<Velocity<Radian>>, endAfterSpinup: Boolean = false) : this({ ShotParameter(hoodAngle, speed) }, endAfterSpinup)

    private val angleEntry: NetworkTableEntry = SmartDashboard.getEntry("hoodAngle")
    private val rpmEntry: NetworkTableEntry = SmartDashboard.getEntry("rpm")

    override fun initialize() {
        angleEntry.setDefaultDouble(45.0)
        rpmEntry.setDefaultDouble(0.0)
    }

    override fun execute() {
//        val wantedParameter = parameterSupplier()
        val wantedParameter = ShotParameter(angleEntry.getDouble(45.0).degrees, rpmEntry.getDouble(0.0).revolutionsPerMinute)

        HoodSubsystem.wantedAngle = wantedParameter.hoodAngle

        // call periodically to recalculate feedback
        FlywheelSubsystem.shootAtSpeed(wantedParameter.speed)
    }

    private fun isOnTarget(): Boolean {
        val wantedParameter = parameterSupplier()
        return abs(wantedParameter.speed.inRpm() - FlywheelSubsystem.flywheelSpeed.inRpm()) < 50 &&
                abs(HoodSubsystem.wantedAngle.inDegrees() - wantedParameter.hoodAngle.inDegrees()) < 3
    }

    override fun isFinished(): Boolean {
        return endAfterSpinup && isOnTarget()
    }

    override fun end(interrupted: Boolean) {
        FlywheelSubsystem.setNeutral()
    }
}
