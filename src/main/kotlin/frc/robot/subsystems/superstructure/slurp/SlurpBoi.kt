package frc.robot.subsystems.superstructure.slurp

import edu.wpi.first.wpilibj.DutyCycle
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.ctre.FalconSRX

object SlurpBoi{
                                    //TODO put real data
    val slurpMotor = FalconSRX(30, NativeUnitRotationModel(4096.nativeUnits * 9.333))

    fun slurp(speed : Double){
    slurpMotor.setDutyCycle(speed)
    }
    fun throwUp(speed: Double){
        slurpMotor.setDutyCycle(-speed)
    }

}




