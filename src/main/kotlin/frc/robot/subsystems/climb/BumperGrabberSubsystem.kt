package frc.robot.subsystems.climb

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.XboxController
import frc.robot.Controls
import frc.robot.Ports.bumperGrabberId
import frc.robot.Ports.bumperGrabberSolenoid
import frc.robot.Ports.kPcmId
import kotlin.properties.Delegates
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.rev.falconMAX
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid

object BumperGrabberSubsystem : FalconSubsystem() {

    private val bumperPiston = FalconDoubleSolenoid(bumperGrabberSolenoid[0], bumperGrabberSolenoid[1], kPcmId)
    val bumperGrabMotor = falconMAX(bumperGrabberId,
            CANSparkMaxLowLevel.MotorType.kBrushless, DefaultNativeUnitModel) {
        canSparkMax.apply {
            restoreFactoryDefaults()
            setSecondaryCurrentLimit(35.0)
        }
        smartCurrentLimit = 25.amps
        brakeMode = true

        controller.setOutputRange(-1.0, 0.0)
    }

    override fun lateInit() {
        wantsExtended = false
    }

    var wantsExtended by Delegates.observable(false,
            { _, _, wantsOut ->
                bumperPiston.state = if (wantsOut) FalconSolenoid.State.Forward else FalconSolenoid.State.Reverse
            })
}

class GrabBumperCommand : FalconCommand(BumperGrabberSubsystem) {

    override fun execute() {
//        // ensure we should be climbing
//        if (!Controls.isClimbing) {
//            cancel()
//            return
//        }

        val speed = -speedSource()

        BumperGrabberSubsystem.wantsExtended = true
        BumperGrabberSubsystem.bumperGrabMotor.setDutyCycle(speed)
    }

    companion object {
        val speedSource by lazy { Controls.operatorFalconXbox.getRawAxis(XboxController.Axis.kRightTrigger.value) }
    }
}
