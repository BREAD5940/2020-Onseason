package frc.robot.subsystems.shooter

import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.Velocity

/**
 * Set the flywheel to shoot at a speed specified by a supplier
 */
class VariableSpeedShootCommand(private val speed: () -> SIUnit<Velocity<Radian>>) : FalconCommand(FlywheelSubsystem) {

    constructor(speed: SIUnit<Velocity<Radian>>) : this({ speed })

    override fun execute() {
        FlywheelSubsystem.setSpeed(speed())
    }

    override fun end(interrupted: Boolean) {
        FlywheelSubsystem.setNeutral()
    }
}
