package frc.robot.subsystems.shooter

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.system.plant.DCMotor
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Ports.armSolenoid
import frc.robot.Ports.collectorAgitatorId
import frc.robot.Ports.kPcmId
import frc.robot.Ports.shooterGearboxIds
import frc.robot.Ports.shooterShifterSolenoid
import kotlin.properties.Delegates
import lib.*
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.lerp
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.*
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.rev.falconMAX
import org.ghrobotics.lib.types.Interpolatable
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid

object FlywheelSubsystem : FalconSubsystem() {

    private val model = NativeUnitRotationModel(0.81.nativeUnits)
    val shooterMaster = falconMAX(shooterGearboxIds[0],
            CANSparkMaxLowLevel.MotorType.kBrushless, model) {
        with(canSparkMax) {
            restoreFactoryDefaults()
        }
        smartCurrentLimit = 50.amps

        outputInverted = true
        brakeMode = false

        controller.setOutputRange(-1.0, 1.0)
        controller.p = 0.0005
        controller.i = 0.0
        controller.d = 0.0006
        controller.ff = 0.0

    }

    private val feedForward = SimpleMotorFeedforward(0.65,  12.0 / 5676.revolutionsPerMinute.value * 0.9)

    private val shooterSlave = falconMAX(shooterGearboxIds[1], CANSparkMaxLowLevel.MotorType.kBrushless, DefaultNativeUnitModel) {
        with(canSparkMax) {
            restoreFactoryDefaults()
        }
        smartCurrentLimit = 50.amps
        controller.setOutputRange(-1.0, 1.0)
        outputInverted = true
        brakeMode = false
        follow(shooterMaster)
    }

    private val shifterSolenoid = FalconDoubleSolenoid(shooterShifterSolenoid[0], shooterShifterSolenoid[1], kPcmId)

    private val armExtensionSolenoid = FalconDoubleSolenoid(
            armSolenoid[0], armSolenoid[1], kPcmId
    )

     val kickWheelMotor = falconMAX(collectorAgitatorId, CANSparkMaxLowLevel.MotorType.kBrushless, DefaultNativeUnitModel) {
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

    fun shootAtSpeed(speed: SIUnit<Velocity<Radian>>) {
        wantsShootMode = true
        val ff = feedForward.calculate(speed.value).volts
        println("shooting at speed ${speed.inRpm()} with ff ${ff.value}")
        shooterMaster.setVelocity(speed, ff)
    }

    fun shootAtPower(power: Double) {
        wantsShootMode = true
        shooterMaster.setDutyCycle(power)
    }

    fun runKickWheel(speed: Double) {
        kickWheelMotor.setDutyCycle(speed)
    }

    fun agitateAndShoot(shootTime: SIUnit<Second> = 5.seconds): CommandBase = sequential {
        +ShootCommand(true)
        +parallel {
            +instantCommand { runKickWheel(0.3) }.perpetually().withTimeout(shootTime.inSeconds())
            +ShootCommand().withTimeout(shootTime.inSeconds())
        }
        +instantCommand { setNeutral() }
    }

    override fun lateInit() {
        SmartDashboard.putData(FlywheelSubsystem)
    }

    /**
     * The default shot lookup table, in degrees of elevation to ShotParameters
     */
    val defaultShotLookupTable = InterpolatingTable(
            // maybe we'll do target pitch for now?
            -3.9 to ShotParameter(67.degrees, 4000.revolutionsPerMinute),
            0.3 to ShotParameter(64.5.degrees, 3500.revolutionsPerMinute, (1).degrees),
            4.3 to ShotParameter(65.degrees, 2600.revolutionsPerMinute, (1).degrees),
            5.4 to ShotParameter(63.8.degrees, 2400.revolutionsPerMinute, (0.5).degrees),
            8.6 to ShotParameter(62.5.degrees, 2400.revolutionsPerMinute, (0.5).degrees),
            12.2 to ShotParameter(61.5.degrees, 2100.revolutionsPerMinute, 0.5.degrees),
            16.2 to ShotParameter(60.5.degrees, 1900.revolutionsPerMinute, 0.5.degrees)
    )

}

data class ShotParameter(val hoodAngle: SIUnit<Radian>, val speed: SIUnit<Velocity<Radian>>,
                         val offset: SIUnit<Radian> = 0.degrees) : Interpolatable<ShotParameter> {

    override fun interpolate(endValue: ShotParameter, t: Double) =
            ShotParameter(SIUnit(hoodAngle.value.lerp(endValue.hoodAngle.value, t)),
                    SIUnit(speed.value.lerp(endValue.speed.value, t)))

    companion object {
        val DefaultParameter = ShotParameter(45.degrees, 5000.revolutionsPerMinute)
    }
}
