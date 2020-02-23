package frc.robot.subsystems.drive.swerve

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState
import kotlin.math.PI
import org.ghrobotics.lib.mathematics.units.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.derived.* // ktlint-disable no-wildcard-imports
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.rev.FalconMAX

open class Mk2SwerveModule(
    azimuthPWMPort: Int,
    azimuthAnalogPort: Int,
    private val offset: SIUnit<Radian>,
    val driveMotor: FalconMAX<Meter>,
    angleKp: Double,
    angleKi: Double,
    angleKd: Double,
    private val angleMotorOutputRange: ClosedFloatingPointRange<Double>
) {

    private val stateMutex = Object()
    val periodicIO = PeriodicIO()
        get() = synchronized(stateMutex) { field }

    val state get() = periodicIO.state
    var output: Output
        get() = periodicIO.desiredOutput
        set(value) { periodicIO.desiredOutput = value }

//    private val azimuthMotor = Spark(azimuthPWMPort)
    public val azimuthMotor = FalconMAX(azimuthPWMPort, CANSparkMaxLowLevel.MotorType.kBrushless, DefaultNativeUnitModel)
    private val azimuthController =
            PIDController(angleKp, angleKi, angleKd).apply {
                //                setInputRange(0.0, 2.0 * PI)
//                setContinuous(true)
//                setOutputRange(-0.5, 0.5)
                enableContinuousInput(-PI, PI)
            }

    private val analogInput = AnalogInput(azimuthAnalogPort)
    val azimuthAngle =
            { ((1.0 - analogInput.voltage / RobotController.getVoltage5V() * 2.0 * PI).radians + offset).toRotation2d() }

    init {
        driveMotor.canSparkMax.restoreFactoryDefaults()
        driveMotor.canSparkMax.setSecondaryCurrentLimit(60.0)
        driveMotor.canSparkMax.setSmartCurrentLimit(35)
        driveMotor.canSparkMax.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false)
        driveMotor.canSparkMax.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false)
        driveMotor.canSparkMax.openLoopRampRate = 0.05

        azimuthMotor.canSparkMax.restoreFactoryDefaults()
        azimuthMotor.canSparkMax.setSecondaryCurrentLimit(20.0)
        azimuthMotor.canSparkMax.setSmartCurrentLimit(30)
        azimuthMotor.canSparkMax.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false)
        azimuthMotor.canSparkMax.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false)

        azimuthMotor.canSparkMax.apply {
//            setSmartCurrentLimit(60)
//            setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 500)
//            setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 3)
        }
    }

    fun updateState() {
//        periodicIO.distance = driveMotor.encoder.position
        periodicIO.state = SwerveModuleState(
                driveMotor.encoder.velocity.value,
                azimuthAngle())
    }

    fun useState() {

        val customizedOutput = customizeAngle(periodicIO.desiredOutput) // TODO reverse
//        azimuthController.setSetpoint(customizedOutput.angle.radians)
        val angleOutput = azimuthController.calculate(
                periodicIO.state.angle.radians, customizedOutput.angle.radians)
        val nextAzimuthOutput = angleOutput.coerceIn(angleMotorOutputRange)

//        azimuthMotor.set(0.2)
//        println("swerve next angle out $nextAzimuthOutput")

        azimuthMotor.setDutyCycle(nextAzimuthOutput)
//        driveMotor.setDutyCycle(0.2)

        periodicIO.lastError = azimuthController.positionError.radians.toRotation2d()
        periodicIO.lastAzimuthOutput = nextAzimuthOutput

//        driveMotor.setDutyCycle(0.0)

//        driveMotor.setNeutral()

//        return

        when (customizedOutput) {
            is Output.Nothing -> {
                driveMotor.setNeutral()
            }
            is Output.Percent -> {
//                println("setting duty cycle ${customizedOutput.percent}")
                driveMotor.setDutyCycle(customizedOutput.percent)
            }
            is Output.Voltage -> {
                driveMotor.setVoltage(customizedOutput.voltage)
            }
            is Output.Velocity -> {
//                driveMotor.setVelocity(customizedOutput.velocity, customizedOutput.arbitraryFeedForward)
                driveMotor.setVoltage(customizedOutput.arbitraryFeedForward)
            }
        }
    }

    /**
     * Decide if we should reverse the module
     * if so, reverse it
     */
    private fun customizeAngle(output: Output): Output {
        val targetAngle = output.angle
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

        /**
         * The last error of the azimuth
         */
        var lastError = Rotation2d()

        /**
         * The last output of the azimuth motor
         */
        var lastAzimuthOutput = 0.0
    }

    sealed class Output(val angle: Rotation2d) {

        abstract fun reverse(): Output

        object Nothing : Output(0.degrees.toRotation2d()) {
            override fun reverse() = this
        }

        class Percent(
            val percent: Double,
            angle: Rotation2d
        ) : Output(angle) {
            override fun reverse(): Output {
                return Percent(-percent, angle + 180.degrees.toRotation2d())
            }
        }

        class Voltage(
            val voltage: SIUnit<Volt>,
            angle: Rotation2d
        ) : Output(angle) {
            override fun reverse(): Output {
                return Voltage(-voltage, angle + 180.degrees.toRotation2d())
            }
        }

        class Velocity(
            val velocity: SIUnit<LinearVelocity>,
            angle: Rotation2d,
            val arbitraryFeedForward: SIUnit<Volt> = 0.volts
        ) : Output(angle) {
            constructor() : this(0.meters.velocity, 0.degrees.toRotation2d(), 0.volts)

            override fun reverse(): Output {
                return Velocity(-velocity, angle + 180.degrees.toRotation2d(), -arbitraryFeedForward)
            }
        }
    }
}
