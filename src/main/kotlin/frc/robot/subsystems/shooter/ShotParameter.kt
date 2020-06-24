package frc.robot.subsystems.shooter

import lib.inRpm
import lib.revolutionsPerMinute
import org.ghrobotics.lib.mathematics.lerp
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.Velocity
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.inDegrees
import org.ghrobotics.lib.types.Interpolatable

data class ShotParameter(
        val hoodAngle: SIUnit<Radian>,
        val speed: SIUnit<Velocity<Radian>>,
        val offset: SIUnit<Radian> = 0.degrees
) : Interpolatable<ShotParameter> {

    override fun interpolate(endValue: ShotParameter, t: Double) =
            ShotParameter(SIUnit(hoodAngle.value.lerp(endValue.hoodAngle.value, t)),
                    SIUnit(speed.value.lerp(endValue.speed.value, t)),
                    SIUnit(offset.value.lerp(endValue.offset.value, t)))

    companion object {
        val defaultParameter = ShotParameter(45.degrees, 5000.revolutionsPerMinute)
    }

    override fun equals(other: Any?): Boolean {
        if (other == null) return false
        if (other !is ShotParameter) return false
        return (other.hoodAngle - hoodAngle).absoluteValue.inDegrees() < 0.1 &&
                (other.speed - speed).value < 0.1 &&
                (other.offset - offset).absoluteValue.inDegrees() < 0.1
    }

    override fun toString() = "Angle ${hoodAngle.inDegrees()} Speed ${speed.inRpm()} Offset ${offset.inDegrees()}"

    override fun hashCode(): Int {
        var result = hoodAngle.hashCode()
        result = 31 * result + speed.hashCode()
        result = 31 * result + offset.hashCode()
        return result
    }
}