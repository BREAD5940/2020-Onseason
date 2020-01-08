package frc.robot.subsystems.superstructure.climb

import com.ctre.phoenix.motorcontrol.InvertType
import frc.robot.subsystems.superstructure.Proximal
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.nativeunit.*
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.motors.ctre.falconSRX
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid

object ClimbSubsystem {

    class Climb(val climbHeight : SIUnit<Meter>): FalconCommand() {

        //TODO Tune PID / Enter Values
        val Kp = 0.00
        val Kd = 0.00
        val solenoid = FalconDoubleSolenoid(0,1, 8)
        //This code was wrote assuming we are useing a "yeet stick" that pulls us up the poll
        val climbMaster: FalconSRX<Meter> = FalconSRX(21, NativeUnitLengthModel(4096.nativeUnits, 0.75.inches))
        val climbSlave = FalconSRX(1   /*CHANGE PORT*/, DefaultNativeUnitModel)
        init{
            //TODO CHECK FOR INVERT!
            climbSlave.follow(climbMaster)
            //climbSlave.talonSRX.setInverted(InvertType.OpposeMaster) //
            climbMaster.talonSRX.config_kP(0,0.0)
            climbMaster.talonSRX.config_kD(0,0.0)
        }
        override fun initialize(){
            climbMaster.setPosition(climbHeight)
        }

        override fun isFinished(): Boolean {
            return (climbMaster.encoder.rawPosition -climbHeight).absoluteValue < 0.05.meters

        }

    }


}


