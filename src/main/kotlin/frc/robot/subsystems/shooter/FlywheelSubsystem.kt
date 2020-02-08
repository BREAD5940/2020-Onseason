package frc.robot.subsystems.shooter

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.robot.Ports.armSolenoid
import frc.robot.Ports.collectorAgitatorId
import frc.robot.Ports.kPcmId
import frc.robot.Ports.shooterGearboxIds
import frc.robot.Ports.shooterShifterSolenoid
import kotlin.math.abs
import kotlin.properties.Delegates
import lib.inRpm
import lib.instantCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.Frac
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.Second
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.derived.*
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.rev.falconMAX
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid

object FlywheelSubsystem : FalconSubsystem() {

    private val shooterMaster = falconMAX(shooterGearboxIds[0],
            CANSparkMaxLowLevel.MotorType.kBrushless, NativeUnitRotationModel(0.81.nativeUnits)) {
        with(canSparkMax) {
            restoreFactoryDefaults()
        }
        controller.p = 0.0
        controller.i = 0.0
        controller.d = 0.0
    }
    private val shooterSlave = falconMAX(shooterGearboxIds[1], CANSparkMaxLowLevel.MotorType.kBrushless, DefaultNativeUnitModel) {
        with(canSparkMax) {
            restoreFactoryDefaults()
        }
        follow(shooterMaster)
    }
    private val shifterSolenoid = FalconDoubleSolenoid(shooterShifterSolenoid[0], shooterShifterSolenoid[1], kPcmId)

    private val armExtensionSolenoid = FalconDoubleSolenoid(
            armSolenoid[0], armSolenoid[1], kPcmId
    )

    private val kickWheelMotor = falconMAX(collectorAgitatorId, CANSparkMaxLowLevel.MotorType.kBrushless, DefaultNativeUnitModel) {
        with(canSparkMax) {
            restoreFactoryDefaults()
            setSecondaryCurrentLimit(35.0)
        }
        smartCurrentLimit = 25.amps
    }

    var wantsShootMode by Delegates.observable(true,
            { _, _, wantsShoot ->
                shifterSolenoid.state = if (wantsShoot) FalconSolenoid.State.Forward else FalconSolenoid.State.Reverse
                // todo switch native unit model?
            })

    fun setClimberArmState(wantsUp: Boolean) {
        armExtensionSolenoid.state = if (wantsUp) FalconSolenoid.State.Forward else FalconSolenoid.State.Reverse
    }

    override fun setNeutral() {
        shooterMaster.setNeutral()
        kickWheelMotor.setNeutral()
    }

    val flywheelSpeed = shooterMaster.encoder.velocity

    // feedforward
    private val feedForward = SimpleMotorFeedforward(0.0, 0.015)

    fun setSpeed(speed: SIUnit<Velocity<Radian>>) {
        wantsShootMode = true
        shooterMaster.setVelocity(speed, feedForward.calculate(speed.value).volts)
    }

    fun runAgitator(speed: Double) {
        kickWheelMotor.setDutyCycle(speed)
    }

    fun agitateAndShoot(speed: () -> SIUnit<Velocity<Radian>>): CommandBase = sequential {
            +WaitUntilCommand { abs(speed().inRpm() - flywheelSpeed.inRpm()) < 40 }
            +instantCommand { runAgitator(0.3) }.perpetually().withTimeout(5.0)
        }.raceWith(VariableSpeedShootCommand(speed))

    fun agitateAndShoot(speed: SIUnit<Frac<Radian, Second>>): CommandBase = agitateAndShoot { speed }
}