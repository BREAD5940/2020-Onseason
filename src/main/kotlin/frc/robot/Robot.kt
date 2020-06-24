package frc.robot

import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.sim.SimFlywheel
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.system.plant.DCMotor
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpiutil.math.Nat
import edu.wpi.first.wpiutil.math.VecBuilder
import frc.robot.autonomous.Autonomous
import frc.robot.subsystems.climb.BumperGrabberSubsystem
import frc.robot.subsystems.drive.DriveSubsystem
import frc.robot.subsystems.intake.IntakeSubsystem
import frc.robot.subsystems.shooter.FlywheelSubsystem
import frc.robot.subsystems.shooter.HoodSubsystem
import frc.robot.subsystems.shooter.ShootCommand
import frc.robot.subsystems.shooter.ShooterController
import frc.robot.subsystems.vision.VisionSubsystem
import frc.team4069.keigen.vec
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.derived.velocity
import org.ghrobotics.lib.wrappers.FalconTimedRobot

object Robot : FalconTimedRobot() {

    const val debugMode = false

    val led = AddressableLED(7)
    val buffer = AddressableLEDBuffer(100) // irl 43

    override fun robotInit() {
        Network // at the top because s3ndable choosers need to be instantiated
        Autonomous

        +DriveSubsystem
        +FlywheelSubsystem
        +IntakeSubsystem
        +HoodSubsystem
        +BumperGrabberSubsystem
        +VisionSubsystem

        SmartDashboard.putData(CommandScheduler.getInstance())

        super.robotInit()

        for (i in 0 until buffer.length) {
            buffer.setRGB(i, 0, 0, 0)
        }
        led.setLength(buffer.length)
        led.setData(buffer)
        led.start()
    }

    var last = 0

    override fun teleopPeriodic() {
    }

    override fun robotPeriodic() {
        Autonomous.update()
        Controls.update()
        Network.update()

        for (i in 0 until buffer.length) {
            buffer.setRGB(i, 0, 0, 0)
        }
        buffer.setRGB(last, 100, 0, 0)
        led.setData(buffer)
        last++
        if (last >= buffer.length) last = 0
    }

    override fun disabledInit() {
    }

    override fun teleopInit() {
        HoodSubsystem.enabledReset()

        ShootCommand({ Constants.rightBelowGoalParameter5v }).schedule()
    }

    override fun autonomousInit() {
        HoodSubsystem.enabledReset()
    }


    override fun simulationInit() {
        flywheelSim.setInput(vec(Nat.N1()).fill(0.0))
    }

    private val flywheelSim = SimFlywheel(ShooterController.plant, true, VecBuilder.fill(0.004),
            DCMotor.getNEO(2), 0.81)

    override fun simulationPeriodic() {
        flywheelSim.setInput(FlywheelSubsystem.shooterMaster.voltageOutput.value)
        flywheelSim.update(0.020)
        FlywheelSubsystem.simVelocity = flywheelSim.getY(0).radians.velocity
    }
}

fun main() {
    Robot.start()
}
