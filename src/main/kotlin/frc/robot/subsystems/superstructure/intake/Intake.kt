package frc.robot.subsystems.superstructure.intake

import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.motors.ctre.falconSPX
import org.ghrobotics.lib.motors.ctre.falconSRX

object Intake{
                        //TODO Gib real ports and stuff
    val intakeMotor = FalconSRX(30, NativeUnitRotationModel(4096.nativeUnits * 9.333))
    //only yeets intake
     fun intakeYeet(){
         intakeMotor.setDutyCycle(1.0)
     }
    fun intakeOutput(){
        intakeMotor.setDutyCycle(-1.0)
    }
}