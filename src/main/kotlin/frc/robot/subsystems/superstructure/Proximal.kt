package frc.robot.subsystems.superstructure

import com.ctre.phoenix.motorcontrol.FeedbackDevice
import com.ctre.phoenix.motorcontrol.InvertType
import edu.wpi.first.wpilibj.controller.ArmFeedforward
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Volt
import org.ghrobotics.lib.mathematics.units.derived.inDegrees
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.derived.volts
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.ctre.falconSRX


object Proximal: FalconSubsystem() {

    val master = falconSRX(30, NativeUnitRotationModel(4096.nativeUnits * 9.33)) {
        with(talonSRX) {
            configFactoryDefault()
            configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative)
            setSensorPhase(true)
        }
        voltageCompSaturation = 12.volts
    }

    val follower = falconSRX(31, DefaultNativeUnitModel) {
        with(talonSRX) {
            configFactoryDefault()
            setInverted(InvertType.OpposeMaster)
        }
        follow(master)
    }

    val position get() = master.encoder.position
    val velocity get() = master.encoder.velocity

    var voltageOutput: SIUnit<Volt>
        get() = master.voltageOutput
        set(value) {
            master.setVoltage(value, 0.volts)
        }

    override fun periodic() {
        SmartDashboard.putNumber("pos", position.inDegrees())
        SmartDashboard.putNumber("volt", voltageOutput.value)
    }
}