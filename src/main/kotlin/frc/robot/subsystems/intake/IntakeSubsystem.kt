package frc.robot.subsystems.intake
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.Controls
import frc.robot.Ports.intakeMotorId
import frc.robot.Ports.intakeSolenoid
import kotlin.properties.Delegates
import lib.instantCommand
import lib.runCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnit
import org.ghrobotics.lib.motors.rev.falconMAX
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid
import java.util.function.DoubleSupplier
import kotlin.math.absoluteValue

object IntakeSubsystem : FalconSubsystem() {
    private const val holdIntake = false
    private val chungusPistonSolenoid = FalconDoubleSolenoid(intakeSolenoid[0], intakeSolenoid[1], 9)
    private val secondarySmolPistonSolenoid = FalconDoubleSolenoid(intakeSolenoid[2], intakeSolenoid[3], 8)

    override fun setNeutral() {
        intakeMotor.setNeutral()
    }

    val intakeMotor = falconMAX(intakeMotorId, CANSparkMaxLowLevel.MotorType.kBrushless, DefaultNativeUnitModel, NativeUnit) {
        canSparkMax.apply {
            restoreFactoryDefaults()
            setSecondaryCurrentLimit(35.0)
        }
        smartCurrentLimit = 20.amps
    }

    fun setSpeed(intakeSpeed: Double) {
        intakeMotor.setDutyCycle(intakeSpeed)
    }

    private var wantsExtended by Delegates.observable(false, { _, _, nowWantsExtended ->
        chungusPistonSolenoid.state = if (nowWantsExtended) FalconSolenoid.State.Forward else FalconSolenoid.State.Reverse
        secondarySmolPistonSolenoid.state = if (nowWantsExtended) FalconSolenoid.State.Forward else FalconSolenoid.State.Reverse }
    )

    fun setChungusPistonExtension(nowWantsExtended: Boolean) {
        chungusPistonSolenoid.state = if (nowWantsExtended) FalconSolenoid.State.Forward else FalconSolenoid.State.Reverse
    }

    fun setSmolPistonExtension(nowWantsExtended: Boolean) {
        secondarySmolPistonSolenoid.state = if (nowWantsExtended) FalconSolenoid.State.Forward else FalconSolenoid.State.Reverse
    }

    fun miniRetractIntakeCommand() = setSmolPistonExtension(false)
    fun miniExtendIntakeCommand() = setSmolPistonExtension(true)
    fun extendIntakeCommand() = sequential {
        +instantCommand(IntakeSubsystem) { setSmolPistonExtension(true); }
        +WaitCommand(0.2)
        +instantCommand(IntakeSubsystem) { setChungusPistonExtension(true) }
    }

    fun retractIntakeCommand() = sequential {
        +instantCommand(IntakeSubsystem) { setChungusPistonExtension(false); }
        +WaitCommand(0.2)
        +instantCommand(IntakeSubsystem) { setSmolPistonExtension(false) }
        +WaitCommand(0.1)
    }

    override fun lateInit() {

        defaultCommand = runCommand({
            val speed = Controls.operatorXbox.getTriggerAxis(GenericHID.Hand.kRight) -
                    Controls.operatorXbox.getTriggerAxis(GenericHID.Hand.kLeft)
            + Controls.driverWpiXbox.getTriggerAxis(GenericHID.Hand.kRight) -
                    Controls.driverWpiXbox.getTriggerAxis(GenericHID.Hand.kLeft)

            setSpeed(speed)
//            if (speed.absoluteValue > 0.1 && !holdIntake) {
//                miniExtendIntakeCommand()
//            } else if (!holdIntake) {
//                miniRetractIntakeCommand()
//            }

        }, this)

        SmartDashboard.putData("retract intake", retractIntakeCommand())
        SmartDashboard.putData("extend intake", extendIntakeCommand())
    }

    override fun periodic() {
//        println(intakeMotor.drawnCurrent.inAmps())
    }
}
