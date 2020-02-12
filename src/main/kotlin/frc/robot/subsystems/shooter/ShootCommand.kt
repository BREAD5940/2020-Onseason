package frc.robot.subsystems.shooter

import frc.robot.subsystems.vision.VisionSubsystem
import lib.inRpm
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.Velocity
import org.ghrobotics.lib.mathematics.units.derived.inDegrees
import kotlin.math.abs

/**
 * Set the flywheel to shoot at a speed specified by a supplier. This command will run
 * forever if [endAfterSpinup] is false, and will exit when the hood and flywheel
 * are spun up and at the correct angle if [endAfterSpinup] is true.
 */
class ShootCommand(private val parameterSupplier: () -> ShotParameter, private val endAfterSpinup: Boolean = false) : FalconCommand(FlywheelSubsystem, HoodSubsystem) {

    constructor(endAfterSpinup: Boolean = false) : this({ FlywheelSubsystem.defaultShotLookupTable.get(VisionSubsystem.yOffset) ?: ShotParameter.DefaultParameter }, endAfterSpinup)

    constructor(hoodAngle: SIUnit<Radian>, speed: SIUnit<Velocity<Radian>>, endAfterSpinup: Boolean = false) : this( { ShotParameter(hoodAngle, speed) }, endAfterSpinup )

    override fun execute() {
        val wantedParameter = parameterSupplier()
        FlywheelSubsystem.shootAtSpeed(wantedParameter.speed)
        HoodSubsystem.wantedAngle = wantedParameter.hoodAngle
    }

    fun isOnTarget(): Boolean {
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
