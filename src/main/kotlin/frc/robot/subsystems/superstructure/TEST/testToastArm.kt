package frc.robot.subsystems.superstructure.TEST

import com.ctre.phoenix.motorcontrol.InvertType
import com.ctre.phoenix.motorcontrol.can.TalonSRX
import edu.wpi.first.wpilibj.Talon
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degree
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.motors.ctre.falconSRX
//import com.revrobotics.CANSensor

object Arm{

    val armMaster = FalconSRX(30, NativeUnitRotationModel(4096.nativeUnits * 9.333))
    val armSlave = FalconSRX(31, DefaultNativeUnitModel)

    class MovePIDTest(private val wantedAngle : SIUnit<Radian>) : FalconCommand(){

        init{
            armMaster.talonSRX.configFactoryDefault()
            armSlave.talonSRX.configFactoryDefault()
            armSlave.follow(armMaster)
            armMaster.talonSRX.config_kP(0, 0.1)
            armMaster.talonSRX.config_kD(0, 0.0)
            armSlave.talonSRX.setInverted(InvertType.OpposeMaster)
        }
        override fun initialize(){
            armMaster.encoder.resetPosition(0.degrees)
            armMaster.setPosition(wantedAngle)
        }
        override fun execute(){

        }
        override fun isFinished(): Boolean {
            return(armMaster.encoder.position -wantedAngle).absoluteValue > 5.degrees
        }
    }

}

