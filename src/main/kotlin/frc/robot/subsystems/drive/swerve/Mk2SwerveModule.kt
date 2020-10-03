package frc.robot.subsystems.drive.swerve

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState
import kotlin.math.PI
import lib.Logger
import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.derived.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.rev.FalconMAX

open class Mk2SwerveModule(
    azimuthMotorCANid: Int,
    azimuthEncoderPort: Int,
    private val offset: SIUnit<Radian>,
    angleKp: Double,
    angleKi: Double,
    angleKd: Double,
    val driveMotor: FalconMAX<Meter>,
    private val angleMotorOutputRange: ClosedFloatingPointRange<Double>,
    name: String
) {

    private val stateMutex = Object() // used to stop multiple threads from accessing the state simultaneously
    val periodicIO = PeriodicIO()
        get() = synchronized(stateMutex) { field }

    val currentState get() = periodicIO.state
    var currentOutput: Output
        get() = periodicIO.desiredOutput
        set(value) { periodicIO.desiredOutput = value }

    val azimuthMotor = FalconMAX(azimuthMotorCANid, CANSparkMaxLowLevel.MotorType.kBrushless, DefaultNativeUnitModel)
    private val azimuthController =
            PIDController(angleKp, angleKi, angleKd).apply {
                enableContinuousInput(-PI, PI)
            }

    private val analogInput = AnalogInput(azimuthEncoderPort)
    val azimuthAngle =
            { ((1.0 - analogInput.voltage / RobotController.getVoltage5V() * 2.0 * PI).radians + offset).toRotation2d() }

    private val logger = Logger("Swerve_$name").apply {
        log("velocity, reference, applied voltage, angle")
    }

    init {
        driveMotor.canSparkMax.restoreFactoryDefaults()
        driveMotor.canSparkMax.setSecondaryCurrentLimit(60.0)
        driveMotor.canSparkMax.setSmartCurrentLimit(35)
        driveMotor.canSparkMax.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false)
        driveMotor.canSparkMax.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false)
        driveMotor.canSparkMax.openLoopRampRate = 0.05
        driveMotor.controller.p = 0.0001 // ' 6e-5 // About 1 order of magnitude below LQR because neo velocity phase lag

        azimuthMotor.canSparkMax.restoreFactoryDefaults()
        azimuthMotor.canSparkMax.setSecondaryCurrentLimit(35.0)
        azimuthMotor.canSparkMax.setSmartCurrentLimit(30)
        azimuthMotor.canSparkMax.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false)
        azimuthMotor.canSparkMax.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false)

        azimuthMotor.canSparkMax.apply {
//            setSmartCurrentLimit(60)
//            setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 500)
//            setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 3)
        }

        logger.log("time, reference, measurement, voltage, current, bus voltage")
    }

    fun updateState() {
//        periodicIO.distance = driveMotor.encoder.position
        periodicIO.state = SwerveModuleState(
                driveMotor.encoder.velocity.value,
                azimuthAngle())
    }

    fun useState() {

        val customizedOutput = customizeAngle(periodicIO.desiredOutput) // Reverse the output, if that's faster

        val angleDutyCycleDemand = azimuthController.calculate(
                periodicIO.state.angle.radians, customizedOutput.moduleAngle.radians)

        val nextAzimuthOutput = angleDutyCycleDemand.coerceIn(angleMotorOutputRange)
        azimuthMotor.setDutyCycle(nextAzimuthOutput)

        when (customizedOutput) {
            is Output.Nothing -> {
                driveMotor.setNeutral()
            }
            is Output.DutyCycle -> {
//                println("setting duty cycle ${customizedOutput.percent}")
                driveMotor.setDutyCycle(customizedOutput.dutyCycle)
            }
            is Output.Voltage -> {
                driveMotor.setVoltage(customizedOutput.voltage)
            }
            is Output.Velocity -> {
                driveMotor.setVelocity(customizedOutput.velocity, customizedOutput.arbitraryFeedForward)
//                driveMotor.setVoltage(customizedOutput.arbitraryFeedForward)
                logger.log(Timer.getFPGATimestamp(), customizedOutput.velocity.inFeetPerSecond(), driveMotor.encoder.velocity.inFeetPerSecond(), driveMotor.voltageOutput.value, driveMotor.drawnCurrent.value, driveMotor.canSparkMax.busVoltage)
            }
        }
    }

    /**
     * Decide if we should reverse the module
     * if so, reverse it
     */
    private fun customizeAngle(output: Output): Output {
        val targetAngle = output.moduleAngle
        val currentAngle = periodicIO.state.angle

        // Deltas that are greater than 90 deg or less than -90 deg can be
        // inverted so the total movement of the module
        // is less than 90 deg by inverting the wheel direction
        val delta = targetAngle - currentAngle
        if (delta.degrees > 90.0 || delta.degrees < -90.0) {
            return output.reverse()
        }
        return output
    }

    class PeriodicIO {
        /**
         * The current state of this module, updating by the [updateState] method.
         */
        var state = SwerveModuleState()

        /**
         * The desired output of this module, which will be sent to the motors
         * in the [useState] method.
         */
        var desiredOutput: Output = Output.Nothing
    }

    sealed class Output(val moduleAngle: Rotation2d) {

        // Reverse the Output. Used if it's faster to reverse the wheel direction then spin
        abstract fun reverse(): Output

        // neutral
        object Nothing : Output(0.degrees.toRotation2d()) {
            override fun reverse() = this
        }

        // duty cycle
        class DutyCycle(
            val dutyCycle: Double,
            angle: Rotation2d
        ) : Output(angle) {
            override fun reverse(): Output {
                return DutyCycle(-dutyCycle, moduleAngle + 180.degrees.toRotation2d())
            }
        }

        class Voltage(
            val voltage: SIUnit<Volt>,
            angle: Rotation2d
        ) : Output(angle) {
            override fun reverse(): Output {
                return Voltage(-voltage, moduleAngle + 180.degrees.toRotation2d())
            }
        }

        class Velocity(
            val velocity: SIUnit<LinearVelocity>,
            angle: Rotation2d,
            val arbitraryFeedForward: SIUnit<Volt> = 0.volts
        ) : Output(angle) {
            constructor() : this(0.meters.velocity, 0.degrees.toRotation2d(), 0.volts)

            override fun reverse(): Output {
                return Velocity(-velocity, moduleAngle + 180.degrees.toRotation2d(), -arbitraryFeedForward)
            }
        }
    }
}
