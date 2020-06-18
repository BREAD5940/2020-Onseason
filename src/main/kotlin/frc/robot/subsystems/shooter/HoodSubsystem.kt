package frc.robot.subsystems.shooter

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile
import frc.robot.Ports
import kotlin.math.PI
import kotlin.math.absoluteValue
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.derived.*
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.rev.falconMAX

object HoodSubsystem : FalconSubsystem() {

    val hoodMotor = falconMAX(Ports.shooterHoodId, CANSparkMaxLowLevel.MotorType.kBrushless, NativeUnitRotationModel((1.0 * 5.0 / 12.0 * 385.0).nativeUnits)) {
        with(canSparkMax) {
            restoreFactoryDefaults()
            setSecondaryCurrentLimit(35.0)
        }
        controller.setOutputRange(-0.3, 0.3)
        controller.p = 0.3
    }

    private val hoodAngleEncoder = AnalogInput(Ports.hoodEncoderPort)
    val hoodAngle
        get() = ((hoodAngleEncoder.voltage / RobotController.getVoltage5V() * 2.0 * PI).radians + 3.degrees) // more offset = lower hood = more curve shot

    var wantedAngle = 55.degrees

    val hoodPidController = PIDController(2.0, 0.0, 0.0).apply {
        //        enableContinuousInput(-PI, PI)
        disableContinuousInput()
    }

    var safeHoodAngles = 17.degrees..78.degrees

    var lastProfiledReference = TrapezoidProfile.State(hoodAngle.value, 0.0)
    private val constraints = TrapezoidProfile.Constraints(75.degrees.inRadians(), 100.degrees.inRadians())

    fun enabledReset() {
        lastProfiledReference = TrapezoidProfile.State(hoodAngle.value, 0.0)
    }

    override fun lateInit() {
        SmartDashboard.putData("hoodPidController", hoodPidController)
    }

    var wasInOnSPARKClosedLoop = false
    var lastBrushlessEncoderAngle = 0.degrees

    override fun periodic() {

        // Calculate the next state by TrapezoidalProfile
        val setpoint = TrapezoidProfile(constraints,
                TrapezoidProfile.State(wantedAngle.coerceIn(safeHoodAngles).value, 0.0),
                lastProfiledReference)
                .calculate(0.020)

        lastProfiledReference = setpoint

        // if we are further than 4 degrees from the endpoint of the TrapezoidalProfile, use our TrapezoidalProfile
        // to set the hood's voltage
        if ((hoodAngle - wantedAngle).absoluteValue > 4.degrees) {
            hoodMotor.setDutyCycle(hoodPidController.calculate(hoodAngle.inRadians(), setpoint.position))
            wasInOnSPARKClosedLoop = false
        } else {
            // if we just entered this mode, set the integrated NEO encoder position to the MA3's position
            if (!wasInOnSPARKClosedLoop) {
                wasInOnSPARKClosedLoop = true
                hoodMotor.encoder.resetPosition(hoodAngle)
            }

            // hold the target angle
            hoodMotor.setPosition(wantedAngle)
        }
    }
}
