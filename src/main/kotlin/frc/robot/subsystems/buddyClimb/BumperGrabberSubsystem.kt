package frc.robot.subsystems.buddyClimb

import com.revrobotics.CANSparkMaxLowLevel
import frc.robot.Ports.bumperGrabberId
import frc.robot.Ports.bumperGrabberSolenoid
import frc.robot.Ports.kPcmId
import kotlin.properties.Delegates
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.rev.falconMAX
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid

object BumperGrabberSubsystem : FalconSubsystem() {

    private val bumperPiston = FalconDoubleSolenoid(bumperGrabberSolenoid[0], bumperGrabberSolenoid[1], kPcmId)
    private val bumperGrabMotor = falconMAX(bumperGrabberId,
            CANSparkMaxLowLevel.MotorType.kBrushless, DefaultNativeUnitModel) {
        canSparkMax.apply {
            restoreFactoryDefaults()
            setSecondaryCurrentLimit(30.0)
        }
        smartCurrentLimit = 25.amps
    }

    var wantsExtended by Delegates.observable(false,
            { _, _, wantsOut ->
                bumperPiston.state = if (wantsOut) FalconSolenoid.State.Forward else FalconSolenoid.State.Reverse
            })
}
