package frc.robot.subsystems.shooter

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile
import frc.robot.Ports
import frc.robot.Robot
import kotlin.math.PI
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.derived.*
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.rev.falconMAX

object HoodSubsystem : FalconSubsystem() {

     val hoodMotor = falconMAX(Ports.shooterHoodId, CANSparkMaxLowLevel.MotorType.kBrushless, DefaultNativeUnitModel) {
        with(canSparkMax) {
            restoreFactoryDefaults()
            setSecondaryCurrentLimit(35.0)
        }
        controller.setOutputRange(-0.1, 0.1)
    }

    private val hoodAngleEncoder = AnalogInput(Ports.hoodEncoderPort)
    val hoodAngle
        get() = ((hoodAngleEncoder.voltage / RobotController.getVoltage5V() * 2.0 * PI).radians + 3.degrees)

    var wantedAngle = 56.degrees

    val hoodPidController = PIDController(5.0, 0.0, 0.0).apply {
//        enableContinuousInput(-PI, PI)
        disableContinuousInput()
    }

    var lastProfiledReference = TrapezoidProfile.State(hoodAngle.value, 0.0)
    private val constraints = TrapezoidProfile.Constraints(20.degrees.inRadians(), 40.degrees.inRadians())

    fun enabledReset() {
        lastProfiledReference = TrapezoidProfile.State(hoodAngle.value, 0.0)
    }

    override fun lateInit() {
        SmartDashboard.putData(hoodPidController)
    }

    override fun periodic() {
        val setpoint = TrapezoidProfile(constraints, TrapezoidProfile.State(wantedAngle.value, 0.0), lastProfiledReference)
                .calculate(0.020)
        lastProfiledReference = setpoint
        hoodMotor.setDutyCycle(hoodPidController.calculate(hoodAngle.inRadians(), setpoint.position))
    }
}
