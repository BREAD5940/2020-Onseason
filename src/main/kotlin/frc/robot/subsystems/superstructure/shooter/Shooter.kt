package frc.robot.subsystems.superstructure.shooter

import com.revrobotics.CANSparkMaxLowLevel
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.Radian
import org.ghrobotics.lib.mathematics.units.derived.Velocity
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.rev.FalconMAX

object Shooter : FalconSubsystem() {
                            //TODO Put in the right values
    val shooterMaster = FalconMAX(1, CANSparkMaxLowLevel.MotorType.kBrushless, NativeUnitRotationModel(4096.nativeUnits * 9.33))
    val shooterSlave = FalconMAX(2, CANSparkMaxLowLevel.MotorType.kBrushless, DefaultNativeUnitModel)
init{
    shooterMaster.canSparkMax.restoreFactoryDefaults()
    shooterSlave.canSparkMax.restoreFactoryDefaults()
    //PID gains, adjust them
        shooterMaster.canSparkMax.pidController.setP(1.0, 1)
        shooterMaster.canSparkMax.pidController.setD(1.0, 1)
        shooterSlave.canSparkMax.pidController.setP(1.0, 1)
        shooterSlave.canSparkMax.pidController.setD(1.0, 1)
    shooterSlave.follow(shooterMaster)
        //TODO Check if inverted
    shooterSlave.outputInverted = false
    shooterMaster.outputInverted = false
}

    fun shootShooter(setSpeed : SIUnit<Velocity<Radian>>) {
        shooterMaster.setVelocity(setSpeed)
    }









}