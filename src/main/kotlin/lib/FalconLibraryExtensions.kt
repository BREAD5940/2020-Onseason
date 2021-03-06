package lib

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.RunCommand
import edu.wpi.first.wpilibj2.command.Subsystem
import kotlin.math.PI
import org.ghrobotics.lib.mathematics.units.SIKey
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.Velocity
import org.ghrobotics.lib.motors.FalconMotor
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.motors.rev.FalconMAX

fun <K : SIKey> FalconMotor<K>.asTalonSRX() = if (this is FalconSRX) this else null

fun <K : SIKey> FalconMotor<K>.asSparkMax() = if (this is FalconMAX) this else null

fun runCommand(block: () -> Unit, vararg reqs: Subsystem) = RunCommand(Runnable(block), *reqs)

fun runCommand(vararg reqs: Subsystem, block: () -> Unit) = RunCommand(Runnable(block), *reqs)

fun instantCommand(block: () -> Unit) = InstantCommand(Runnable(block))

fun instantCommand(vararg reqs: Subsystem, block: () -> Unit) = InstantCommand(Runnable(block), *reqs)

fun SIUnit<Velocity<Radian>>.inRpm() = this.value / 2.0 / PI * 60.0

val Number.revolutionsPerMinute: SIUnit<Velocity<Radian>>
    get() = SIUnit(this.toDouble() /* so this is currently in rpm so div by 60 to get rps */ / 60.0 * 2 * PI)

fun Command.beforeStarting(block: () -> Unit) = this.beforeStarting(Runnable { block() })
