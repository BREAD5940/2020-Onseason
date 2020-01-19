package frc.robot.subsystems.superstructure.shooter

import com.revrobotics.CANSparkMaxLowLevel
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.rev.FalconMAX

object Shooter : FalconSubsystem() {
    //TODO Put in the right values
    private val topShooterMotor = FalconMAX(20, CANSparkMaxLowLevel.MotorType.kBrushless, NativeUnitRotationModel(4096.nativeUnits * 9.33))
    private val lowerShooterMotor = FalconMAX(21, CANSparkMaxLowLevel.MotorType.kBrushless, DefaultNativeUnitModel)

    init{
        topShooterMotor.canSparkMax.restoreFactoryDefaults()
        lowerShooterMotor.canSparkMax.restoreFactoryDefaults()
        //PID gains, adjust them
        topShooterMotor.canSparkMax.pidController.setP(0.0, 0)
        topShooterMotor.canSparkMax.pidController.setD(0.0, 0)
        lowerShooterMotor.canSparkMax.pidController.setP(0.0, 0)
        lowerShooterMotor.canSparkMax.pidController.setD(0.0, 0)
//    shooterSlave.follow(shooterMaster)
        //TODO Check if inverted
        lowerShooterMotor.outputInverted = false
        topShooterMotor.outputInverted = false

        topShooterMotor.canSparkMax.setOpenLoopRampRate(2.0)
        lowerShooterMotor.canSparkMax.setOpenLoopRampRate(2.0)
    }

    fun shoot(speed: Double) {
        topShooterMotor.setDutyCycle(speed)
        lowerShooterMotor.setDutyCycle(speed)
    }

    override fun setNeutral() {
        topShooterMotor.setNeutral()
        lowerShooterMotor.setNeutral()
    }







}