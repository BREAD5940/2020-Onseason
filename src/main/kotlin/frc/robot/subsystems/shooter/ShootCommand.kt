package frc.robot.subsystems.shooter

import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.subsystems.vision.VisionSubsystem
import lib.inRpm
import lib.revolutionsPerMinute
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.Velocity
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.inDegrees
import kotlin.math.abs

/**
 * Set the flywheel to shoot at a speed specified by a supplier. This command will run
 * forever if [endAfterSpinup] is false, and will exit when the hood and flywheel
 * are spun up and at the correct angle if [endAfterSpinup] is true.
 */
class ShootCommand(private val parameterSupplier: () -> ShotParameter, private val endAfterSpinup: Boolean = false) : FalconCommand(FlywheelSubsystem, HoodSubsystem) {

    constructor(endAfterSpinup: Boolean = false) : this({ FlywheelSubsystem.defaultShotLookupTable.get(VisionSubsystem.ps3eye.yaw.degrees) ?: ShotParameter.DefaultParameter }, endAfterSpinup)

    constructor(hoodAngle: SIUnit<Radian>, speed: SIUnit<Velocity<Radian>>, endAfterSpinup: Boolean = false) : this( { ShotParameter(hoodAngle, speed) }, endAfterSpinup )

    private val angleEntry: NetworkTableEntry = SmartDashboard.getEntry("hoodAngle")
    private val rpmEntry: NetworkTableEntry = SmartDashboard.getEntry("rpm")

    override fun initialize() {
        angleEntry.setDefaultDouble(45.0)
        rpmEntry.setDefaultDouble(0.0)
    }

    override fun execute() {
        val wantedParameter = parameterSupplier()
//        val wantedParameter = ShotParameter(angleEntry.getDouble(45.0).degrees, rpmEntry.getDouble(0.0).revolutionsPerMinute)

        FlywheelSubsystem.shootAtSpeed(wantedParameter.speed)
        HoodSubsystem.wantedAngle = wantedParameter.hoodAngle

//        println("hood ${wantedParameter.hoodAngle.inDegrees()}, rpm ${wantedParameter.speed.inRpm()}")
    }

    private fun isOnTarget(): Boolean {
        val wantedParameter = parameterSupplier()
        return abs(wantedParameter.speed.inRpm() - FlywheelSubsystem.flywheelSpeed.inRpm()) < 50
                && abs(HoodSubsystem.wantedAngle.inDegrees() - wantedParameter.hoodAngle.inDegrees()) < 3
    }

    override fun isFinished(): Boolean {
        return endAfterSpinup && isOnTarget()
    }

    override fun end(interrupted: Boolean) {
        FlywheelSubsystem.setNeutral()
    }
}
