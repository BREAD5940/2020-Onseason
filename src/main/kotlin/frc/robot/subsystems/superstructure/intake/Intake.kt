package frc.robot.subsystems.superstructure.intake
import com.revrobotics.CANSparkMaxLowLevel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.rev.FalconMAX

object Intake{
                        //TODO Gib real ports and stuff
    val intakeMotor = FalconMAX(30, CANSparkMaxLowLevel.MotorType.kBrushless, NativeUnitRotationModel(4096.nativeUnits * 9.333))
    //only yeets intake
     fun intakeYeet(intakeSpeed: Double){
         intakeMotor.setDutyCycle(intakeSpeed)
     }


    fun intakeOutput(){
        intakeMotor.setDutyCycle(-0.8)
    }

}