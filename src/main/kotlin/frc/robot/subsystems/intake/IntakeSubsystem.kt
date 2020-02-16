package frc.robot.subsystems.intake
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.Controls
import frc.robot.Ports.intakeMotorId
import frc.robot.Ports.intakeSolenoid
import lib.instantCommand
import kotlin.properties.Delegates
import lib.runCommand
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.inAmps
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.rev.falconMAX
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid

object IntakeSubsystem : FalconSubsystem() {
    val open = false
    val miniOpen = false
    var holdIntake = false
    val chungusPistonSolenoid = FalconDoubleSolenoid(intakeSolenoid[0], intakeSolenoid[1], 9)
    val secondarySmolPistonSolenoid = FalconDoubleSolenoid(intakeSolenoid[2], intakeSolenoid[3], 8)

     val intakeMotor = falconMAX(intakeMotorId, CANSparkMaxLowLevel.MotorType.kBrushless, DefaultNativeUnitModel) {
        canSparkMax.apply {
            restoreFactoryDefaults()
            setSecondaryCurrentLimit(30.0)
        }
        smartCurrentLimit = 25.amps
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

    private fun setSmolPistonExtension(nowWantsExtended: Boolean) {
        secondarySmolPistonSolenoid.state = if (nowWantsExtended) FalconSolenoid.State.Forward else FalconSolenoid.State.Reverse
    }
    fun toggleIntakeExtensionCommand() {
        if(open){
            retractIntakeCommand()
        }
        else {
            extendIntakeCommand()
        }
    }
    fun toggleMiniIntakeExtensionCommand(){
        if(miniOpen){
            miniRetractIntakeCommand()
        }
        else {
            miniExtendIntakeCommand()
        }
    }


     fun miniRetractIntakeCommand() = setSmolPistonExtension(false)

     fun miniExtendIntakeCommand() = setSmolPistonExtension(true);



     fun extendIntakeCommand() = sequential {
        +instantCommand(IntakeSubsystem) { setSmolPistonExtension(true); }
         +instantCommand(IntakeSubsystem) { intakeMotor.setDutyCycle(0.5)}
        +WaitCommand(0.2)
         +instantCommand { IntakeSubsystem.intakeMotor.setNeutral() }
        +instantCommand(IntakeSubsystem) { setChungusPistonExtension(true) }
    }

     fun retractIntakeCommand() = sequential {
            +instantCommand(IntakeSubsystem) { setChungusPistonExtension(false);  }
         +instantCommand(IntakeSubsystem) { intakeMotor.setDutyCycle(-0.5)}
            +WaitCommand(0.2)
         +instantCommand(IntakeSubsystem) { intakeMotor.setNeutral()}
            +instantCommand(IntakeSubsystem) { setSmolPistonExtension(false) }
        }

    override fun lateInit() {
        defaultCommand = runCommand({
            setSpeed(speedSource())
            if(speedSource() > 0.1 && !holdIntake){
                miniExtendIntakeCommand()
            }else if(!holdIntake){
                miniRetractIntakeCommand()
            }
        }, this)

        SmartDashboard.putData("retract intake", retractIntakeCommand())
        SmartDashboard.putData("extend intake", extendIntakeCommand())
    }

    override fun periodic() {
//        println(intakeMotor.drawnCurrent.inAmps())
    }

    // Operator joystick memes
    val speedSource by lazy {
        { Controls.operatorXbox.getTriggerAxis(GenericHID.Hand.kRight) -
                Controls.operatorXbox.getTriggerAxis(GenericHID.Hand.kLeft) }
    }
}

