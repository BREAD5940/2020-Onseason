package lib

import org.ghrobotics.lib.mathematics.units.SIKey
import org.ghrobotics.lib.motors.FalconMotor
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.motors.rev.FalconMAX

fun <K : SIKey> FalconMotor<K>.asTalonSRX() = if (this is FalconSRX) this else null

fun <K : SIKey> FalconMotor<K>.asSparkMax() = if (this is FalconMAX) this else null
