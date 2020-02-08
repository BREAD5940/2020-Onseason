package frc.robot.subsystems.intake
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.GenericHID
import frc.robot.Controls
import frc.robot.Ports.intakeMotorId
import frc.robot.Ports.intakeSolenoid
import frc.robot.Ports.kPcmId
import kotlin.properties.Delegates
import lib.runCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.rev.falconMAX
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid

object IntakeSubsystem : FalconSubsystem() {

    private val solenoid = FalconDoubleSolenoid(intakeSolenoid[0], intakeSolenoid[1], 9)
    private val intakeMotor = falconMAX(intakeMotorId, CANSparkMaxLowLevel.MotorType.kBrushless, DefaultNativeUnitModel) {
        canSparkMax.apply {
            restoreFactoryDefaults()
            setSecondaryCurrentLimit(30.0)
        }
        smartCurrentLimit = 25.amps
    }

    fun setSpeed(intakeSpeed: Double) {
        intakeMotor.setDutyCycle(intakeSpeed)
    }

    var wantsExtended by Delegates.observable(true, { _, _, nowWantsExtended ->
        solenoid.state = if (nowWantsExtended) FalconSolenoid.State.Forward else FalconSolenoid.State.Reverse })

    override fun lateInit() {
        defaultCommand = runCommand({ setSpeed(speedSource()) }, this)
    }

    // Operator joystick memes
    val speedSource by lazy {
        { Controls.operatorXbox.getTriggerAxis(GenericHID.Hand.kRight) -
                Controls.operatorXbox.getTriggerAxis(GenericHID.Hand.kLeft) }
    }
}
