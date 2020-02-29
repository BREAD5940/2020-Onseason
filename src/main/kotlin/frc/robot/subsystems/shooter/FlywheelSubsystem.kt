package frc.robot.subsystems.shooter

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.LinearFilter
import edu.wpi.first.wpilibj.Servo
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.Ports.armSolenoid
import frc.robot.Ports.collectorAgitatorId
import frc.robot.Ports.kPcmId
import frc.robot.Ports.shooterGearboxIds
import frc.robot.Ports.shooterShifterSolenoid
import kotlinx.coroutines.GlobalScope
import lib.inRpm
import lib.instantCommand
import lib.revolutionsPerMinute
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
import org.ghrobotics.lib.utils.launchFrequency
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid
import kotlin.math.PI
import kotlin.properties.Delegates

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

    private val armLimitSwitchDIO = DigitalInput(8)
    val armLimitTriggered get() = !armLimitSwitchDIO.get()

    val kickWheelMotor = falconMAX(collectorAgitatorId, CANSparkMaxLowLevel.MotorType.kBrushless, DefaultNativeUnitModel) {
        with(canSparkMax) {
            restoreFactoryDefaults()
            setSecondaryCurrentLimit(35.0)
        }
        smartCurrentLimit = 25.amps
    }

    val pawlServo = Servo(8).apply {
        setBounds(2.1, 0.0, 0.0, 0.0, 0.9)
    }

    val throughBoreEncoder = Encoder(0, 1).apply {
        distancePerPulse = 2.0 * PI / 0.81 / 2048.0
    }

    fun engagePawl() {
        pawlServo.set(0.8) // random number because of unknown reason, but engagement is good. between 0 and 1
    }

    fun disengagePawl() {
        pawlServo.set(0.0)
    }

    var wantsShootMode by Delegates.observable(true,
            { _, _, wantsShoot ->
                shifterSolenoid.state = if (wantsShoot) FalconSolenoid.State.Forward else FalconSolenoid.State.Reverse
                // todo switch native unit model?
            })

    val armExtended get() = armExtensionSolenoid.state == FalconSolenoid.State.Forward
    fun setClimberArmExtension(wantsUp: Boolean) {
        armExtensionSolenoid.state = if (wantsUp) FalconSolenoid.State.Forward else FalconSolenoid.State.Reverse
    }

    override fun setNeutral() {
        shooterMaster.setNeutral()
        kickWheelMotor.setNeutral()
    }

    val flywheelSpeed
//        get() = shooterMaster.encoder.velocity
        get() = -throughBoreEncoder.rate.radians.velocity

    val filter = LinearFilter.movingAverage(8)
    var smoothedFlywheelSpeed = 0.radians.velocity
        @Synchronized get
        @Synchronized set

    val updateJob = GlobalScope.launchFrequency(400) {
        smoothedFlywheelSpeed = filter.calculate(-throughBoreEncoder.rate).radians.velocity
    }

    override fun periodic() {
        if(!updateJob.isActive) updateJob.start()
    }

    /**
     * Shoot at a speed. Must be called periodically!
     */
    fun shootAtSpeed(speed: SIUnit<Velocity<Radian>>) {
        wantsShootMode = true
        val ff = feedForward.calculate(speed.value).volts
        val fb = feedBack.calculate(smoothedFlywheelSpeed.value, speed.value).volts
        shooterMaster.setVoltage(fb, ff)
//        shooterMaster.setVelocity(speed, ff)
    }

    fun shootAtPower(power: Double) {
        wantsShootMode = true
        shooterMaster.setDutyCycle(power)
    }

    fun shootAtVoltage(volts: SIUnit<Volt>) {
        wantsShootMode = true
        shooterMaster.setVoltage(volts)
    }

    fun runKickWheel(speed: Double) {
        kickWheelMotor.setDutyCycle(speed)
    }

    fun climbAtPower(power: Double) {
        wantsShootMode = false
        shooterMaster.setDutyCycle(power)
    }

    fun agitateAndShoot(shootTime: SIUnit<Second> = 5.seconds): CommandBase = sequential {
        +ShootCommand(true).withTimeout(3.0) // TODO make less bad
        +parallel {
            +instantCommand { runKickWheel(0.8) }.perpetually().withTimeout(shootTime.inSeconds())
            +ShootCommand().withTimeout(shootTime.inSeconds())
        }
        +instantCommand(this@FlywheelSubsystem) { setNeutral() }
    }

    override fun lateInit() {
        SmartDashboard.putData(FlywheelSubsystem)
        SmartDashboard.putData("flywheel PID", feedBack)
        engagePawl()
        enableMotors()
        wantsShootMode = true
        setClimberArmExtension(false)
    }

    fun disableMotors() {
        shooterMaster.controller.setOutputRange(0.0, 0.0)
    }

    fun enableMotors() {
        shooterMaster.controller.setOutputRange(-1.0, 1.0)
    }

    val defaultShotLookupTable = Constants.distanceLookupTable5v

    // STATE SPACE STUFF
    private val feedForward = SimpleMotorFeedforward(0.6, 12.0 / 5676.revolutionsPerMinute.value * 0.9)
    private val feedBack = PIDController(0.02, 0.0, 0.0)

}

data class ShotParameter(
    val hoodAngle: SIUnit<Radian>,
    val speed: SIUnit<Velocity<Radian>>,
    val offset: SIUnit<Radian> = 0.degrees
) : Interpolatable<ShotParameter> {

    override fun interpolate(endValue: ShotParameter, t: Double) =
            ShotParameter(SIUnit(hoodAngle.value.lerp(endValue.hoodAngle.value, t)),
                    SIUnit(speed.value.lerp(endValue.speed.value, t)),
                    SIUnit(offset.value.lerp(endValue.offset.value, t)))

    companion object {
        val DefaultParameter = ShotParameter(45.degrees, 5000.revolutionsPerMinute)
    }

    override fun equals(other: Any?): Boolean {
        if (other == null) return false
        if (other !is ShotParameter) return false
        return (other.hoodAngle - hoodAngle).absoluteValue.inDegrees() < 0.1 &&
                (other.speed - speed).value < 0.1 &&
                (other.offset - offset).absoluteValue.inDegrees() < 0.1
    }

    override fun toString() = "ShotParameter: angle ${hoodAngle.inDegrees()}, speed ${speed.inRpm()}, offset ${offset.inDegrees()}"

    override fun hashCode(): Int {
        var result = hoodAngle.hashCode()
        result = 31 * result + speed.hashCode()
        result = 31 * result + offset.hashCode()
        return result
    }
}
