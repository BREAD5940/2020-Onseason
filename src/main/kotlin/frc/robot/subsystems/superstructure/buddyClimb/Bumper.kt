package frc.robot.subsystems.superstructure.buddyClimb

import com.revrobotics.CANSparkMaxLowLevel
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid

object Bumper {
    val bumperGrabMotor = FalconMAX(21, CANSparkMaxLowLevel.MotorType.kBrushless, DefaultNativeUnitModel)
    val bumperPiston = FalconDoubleSolenoid(1,0, 8)
   init{
       bumperPiston.state = FalconSolenoid.State.Reverse
   }
    fun grab(speed: Double){
        bumperPiston.state = FalconSolenoid.State.Forward
        bumperGrabMotor.setDutyCycle(speed)
    }
    fun stop(){
        bumperGrabMotor.setDutyCycle(0.05)
    }


}