package frc.robot.subsystems.shooter

import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.AnalogInput
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Ports
import kotlin.math.PI
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.derived.*
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.rev.falconMAX

object HoodSubsystem : FalconSubsystem() {

    private val hoodMotor = falconMAX(Ports.shooterHoodId, CANSparkMaxLowLevel.MotorType.kBrushless, DefaultNativeUnitModel) {
        with(canSparkMax) {
            restoreFactoryDefaults()
            setSecondaryCurrentLimit(35.0)
        }
        controller.setOutputRange(-1.0, 1.0)
    }

    private val hoodAngleEncoder = AnalogInput(Ports.hoodEncoderPort)
    private val hoodAngle
        get() = ((1.0 - hoodAngleEncoder.voltage / RobotController.getVoltage5V() * 2.0 * PI).radians + 0.degrees).toRotation2d()

    var wantedAngle = 45.degrees

    private val hoodPidController = PIDController(0.1, 0.0, 0.0)

    override fun lateInit() {
        SmartDashboard.putData(hoodPidController)
    }

    override fun periodic() {
        hoodMotor.setDutyCycle(hoodPidController.calculate(hoodAngle.radians, wantedAngle.inRadians()))
    }
}
