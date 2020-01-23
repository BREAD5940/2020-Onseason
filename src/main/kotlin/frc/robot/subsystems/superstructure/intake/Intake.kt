package frc.robot.subsystems.superstructure.intake
import com.revrobotics.CANSparkMaxLowLevel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid

object Intake{
                        //TODO Gib real ports and stuff
    val intakeMotor = FalconMAX(30, CANSparkMaxLowLevel.MotorType.kBrushless, NativeUnitRotationModel(4096.nativeUnits * 9.333))
    val solenoid = FalconDoubleSolenoid(0, 1, 8)
    //YEET SKEET no u
    //intake done!

     fun intakeSpeed(intakeSpeed: Double){
         intakeMotor.setDutyCycle(-intakeSpeed)
     }
    fun intakeUp(){
        solenoid.state = FalconSolenoid.State.Reverse
    }
    fun intakeDown(){
        solenoid.state = FalconSolenoid.State.Forward
    }

}