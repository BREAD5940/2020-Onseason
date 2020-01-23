package frc.robot.subsystems.superstructure.shooter

import com.revrobotics.CANSparkMaxLowLevel
import frc.robot.subsystems.superstructure.climb.ClimbSubsystem

import frc.robot.subsystems.superstructure.climb.ClimbSubsystem.PTOsolenoid
import org.ghrobotics.lib.commands.FalconCommand

import org.ghrobotics.lib.mathematics.units.nativeunit.DefaultNativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.NativeUnitRotationModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.ghrobotics.lib.wrappers.FalconSolenoid

object Shooter : FalconCommand() {
    //TODO Put in the right values
    private val topShooterMotor = FalconMAX(20, CANSparkMaxLowLevel.MotorType.kBrushless, NativeUnitRotationModel(4096.nativeUnits * 9.33))
    private val lowerShooterMotor = FalconMAX(21, CANSparkMaxLowLevel.MotorType.kBrushless, DefaultNativeUnitModel)

    init{
        //TODO This is bad, but the tunings in climb will stay so just tune there
//        topShooterMotor.canSparkMax.restoreFactoryDefaults()
//        lowerShooterMotor.canSparkMax.restoreFactoryDefaults()
//        //PID gains, adjust them
//        topShooterMotor.canSparkMax.pidController.setP(0.0, 0)
//        topShooterMotor.canSparkMax.pidController.setD(0.0, 0)
//        lowerShooterMotor.canSparkMax.pidController.setP(0.0, 0)
//        lowerShooterMotor.canSparkMax.pidController.setD(0.0, 0)
//        PTOsolenoid.state = FalconSolenoid.State.Reverse
////    shooterSlave.follow(shooterMaster)
//        //TODO Check if inverted
//        lowerShooterMotor.outputInverted = false
//        topShooterMotor.outputInverted = false
        PTOsolenoid.state = FalconSolenoid.State.Forward
//        topShooterMotor.canSparkMax.setOpenLoopRampRate(2.0)
//        lowerShooterMotor.canSparkMax.setOpenLoopRampRate(2.0)
    }

    fun shoot(speed: Double) {
        if(PTOsolenoid.state == FalconSolenoid.State.Forward) {
            ClimbSubsystem.climbShooterMaster.setDutyCycle(speed)
        }
    }

    fun setNeutral() {
        topShooterMotor.setNeutral()
        lowerShooterMotor.setNeutral()
    }







}