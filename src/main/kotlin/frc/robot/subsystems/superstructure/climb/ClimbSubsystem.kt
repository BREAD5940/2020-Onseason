package frc.robot.subsystems.superstructure.climb

import com.ctre.phoenix.motorcontrol.InvertType
import com.revrobotics.CANSparkMaxLowLevel
import frc.robot.subsystems.superstructure.Proximal
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.nativeunit.*
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.motors.ctre.falconSRX
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid

object ClimbSubsystem {

    class Climb(val climbHeight : SIUnit<Meter>): FalconCommand() {

        //TODO Tune PID / Enter Values
        val Kp = 0.00
        val Kd = 0.00
        val solenoid = FalconDoubleSolenoid(0,1, 8)
        //This code was wrote assuming we are useing a "yeet stick" that pulls us up the poll
        val climbMaster: FalconMAX<Meter> = FalconMAX(21, CANSparkMaxLowLevel.MotorType.kBrushless, NativeUnitLengthModel(4096.nativeUnits, 0.75.inches))
        val climbSlave = FalconMAX(1   /*CHANGE PORT*/, CANSparkMaxLowLevel.MotorType.kBrushless, DefaultNativeUnitModel)
        init{
            //TODO CHECK FOR INVERT!
            climbMaster.canSparkMax.restoreFactoryDefaults()
            climbSlave.canSparkMax.restoreFactoryDefaults()
            climbSlave.follow(climbMaster)
            //TODO Tune PID and test inverted
            //climbSlave.talonSRX.setInverted(InvertType.OpposeMaster) //
            climbMaster.canSparkMax.pidController.setP(1.0, 1)
            climbMaster.canSparkMax.pidController.setD(1.0, 1)
            climbSlave.canSparkMax.pidController.setP(1.0, 1)
            climbMaster.canSparkMax.pidController.setD(1.0, 1)
            
        }
        override fun initialize(){
            climbMaster.setPosition(climbHeight)
        }

        override fun isFinished(): Boolean {
            return (climbMaster.encoder.position -climbHeight).absoluteValue < 0.05.meters

        }

    }


}


