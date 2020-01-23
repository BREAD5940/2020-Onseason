package frc.robot.subsystems.superstructure.climb
import com.revrobotics.CANSparkMaxLowLevel
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.nativeunit.*
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.ghrobotics.lib.wrappers.FalconDoubleSolenoid
import org.ghrobotics.lib.wrappers.FalconSolenoid

object ClimbSubsystem {
    val PTOsolenoid = FalconDoubleSolenoid(0, 1, 8)
    val climbShooterMaster: FalconMAX<Meter> = FalconMAX(21, CANSparkMaxLowLevel.MotorType.kBrushless, NativeUnitLengthModel(4096.nativeUnits, 0.75.inches))
    class Climb(val climbHeight : SIUnit<Meter>): FalconCommand() {
//DONE!!
        //TODO Tune PID / Enter Values
        val Kp = 0.00
        val Kd = 0.00
        //TODO Change these pistons, and shifters!!!!!!
        val solenoid = FalconDoubleSolenoid(0, 1, 8)

        //This code was wrote assuming we are useing a "yeet stick" that pulls us up the poll TODO Tune this!!
       // val climbShooterMaster: FalconMAX<Meter> = FalconMAX(21, CANSparkMaxLowLevel.MotorType.kBrushless, NativeUnitLengthModel(4096.nativeUnits, 0.75.inches))
        val climbSlave = FalconMAX(1   /*CHANGE PORT*/, CANSparkMaxLowLevel.MotorType.kBrushless, DefaultNativeUnitModel)
        //val climbSlave2 = FalconMAX(3   /*CHANGE PORT*/, CANSparkMaxLowLevel.MotorType.kBrushless, DefaultNativeUnitModel)

        init {
            //TODO CHECK FOR INVERT!
            climbShooterMaster.canSparkMax.restoreFactoryDefaults()
            climbSlave.canSparkMax.restoreFactoryDefaults()
           // climbSlave2.canSparkMax.restoreFactoryDefaults()
            //TODO Check for inverted!!!
            climbSlave.follow(climbShooterMaster)
            //climbSlave2.follow(climbMaster)
            PTOsolenoid.state = FalconSolenoid.State.Reverse
            //TODO Tune PID and test inverted

            climbShooterMaster.canSparkMax.pidController.setP(1.0, 1)
            climbShooterMaster.canSparkMax.pidController.setD(1.0, 1)
            climbSlave.canSparkMax.pidController.setP(1.0, 1)
            climbSlave.canSparkMax.pidController.setD(1.0, 1)
            //climbSlave2.canSparkMax.pidController.setP(1.0, 1)
            //climbSlave2.canSparkMax.pidController.setD(1.0, 1)
        }

        override fun initialize() {
            if(PTOsolenoid.state == FalconSolenoid.State.Reverse){
                climbShooterMaster.setPosition(climbHeight)
            }

        }

        override fun isFinished(): Boolean {
            if((climbShooterMaster.encoder.position - climbHeight).absoluteValue < 0.05.meters) {
                solenoid.state = FalconSolenoid.State.Reverse
                return true
            } else {
                return false
            }
        }

    }
}


