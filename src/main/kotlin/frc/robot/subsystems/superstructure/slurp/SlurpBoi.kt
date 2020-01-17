package frc.robot.subsystems.superstructure.slurp
import com.revrobotics.CANSparkMaxLowLevel
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.motors.rev.FalconMAX

object SlurpBoi{
    //TODO put real data
    val slurpMotor = FalconMAX(20, CANSparkMaxLowLevel.MotorType.kBrushless, DefaultNativeUnitModel)

    fun slurp(speed : Double){
        slurpMotor.setDutyCycle(speed)
        //on the outside is slurp slurp on the inside i hurt hurt
    }
    fun throwUp(speed: Double){
        slurpMotor.setDutyCycle(-speed)
    }

}




