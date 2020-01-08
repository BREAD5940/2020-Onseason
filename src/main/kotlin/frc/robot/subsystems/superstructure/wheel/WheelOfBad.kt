package frc.robot.subsystems.superstructure.wheel

import edu.wpi.first.wpilibj.Solenoid
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.ctre.FalconSRX
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid

object ControlPanel{
    //TODO Tune this plz
    val wheelMotor = FalconSRX(1, NativeUnitRotationModel(4096.nativeUnits * 8))
    val arm = FalconDoubleSolenoid(0,1, 8)
class SpinThatWheel() : FalconSubsystem(){
    //Assuming that the encoder is a a 1-1 with the axel
    val WheelSize = 4
    val DistanceFromCenter = 5
    val RotationsToGoal = ((DistanceFromCenter*Math.PI)/WheelSize)


    fun initalize(){

    }
    fun isFinished(){

    }




}

}