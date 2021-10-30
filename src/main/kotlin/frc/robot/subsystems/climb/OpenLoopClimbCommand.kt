package frc.robot.subsystems.climb

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.Controls
import frc.robot.subsystems.shooter.FlywheelSubsystem
import kotlin.math.absoluteValue
import lib.instantCommand
import lib.runCommand
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.utils.withDeadband
import org.ghrobotics.lib.wrappers.hid.getY

class OpenLoopClimbCommand : FalconCommand(FlywheelSubsystem) {

    override fun initialize() {
        FlywheelSubsystem.wantsShootMode = false
        FlywheelSubsystem.setClimberArmExtension(true)
        FlywheelSubsystem.engagePawl()
        FlywheelSubsystem.shooterMaster.controller.setOutputRange(-1.0, 0.0)
        SmartDashboard.putBoolean("CLIMB IN DANGER", false)
        println("Starting climb command")
    }

    override fun execute() {
        if (FlywheelSubsystem.armLimitTriggered) {
            FlywheelSubsystem.climbAtPower(0.0)
            return
        }

        if (FlywheelSubsystem.shooterMaster.encoder.position < 4000.degrees) { // this prevents us from climbing too far maybe
//            cancel() // I don't like hard coded constants but ok
//            FlywheelSubsystem.climbAtPower(0.0)
//            return
            FlywheelSubsystem.climbAtPower((speedSource() / 5.0).coerceIn(-0.1, 0.0))
            SmartDashboard.putBoolean("CLIMB IN DANGER", true)
        }
        FlywheelSubsystem.climbAtPower(speedSource().coerceIn(-1.0, 0.0))
    }

    override fun end(interrupted: Boolean) {
        FlywheelSubsystem.engagePawl()
        FlywheelSubsystem.disableMotors()
        FlywheelSubsystem.shooterMaster.brakeMode = false
    }

    override fun isFinished() = FlywheelSubsystem.armLimitTriggered;

    companion object {
        val speedSource by lazy { Controls.operatorFalconXbox.getY(GenericHID.Hand.kRight).withDeadband(0.15) }
    }
}

val openLoopClimbCommandGroup
    get() = sequential {
        +instantCommand {
            BumperGrabberSubsystem.wantsExtended = true
        }
        +WaitCommand(1.3)
        +instantCommand(FlywheelSubsystem) {
            FlywheelSubsystem.shooterMaster.encoder.resetPosition(0.radians) // zero the encoder
            FlywheelSubsystem.wantsShootMode = false
            FlywheelSubsystem.setClimberArmExtension(true)
        }
        +WaitCommand(0.5).andThen { FlywheelSubsystem.disengagePawl() }.andThen(
                WaitCommand(0.5)
        )

        // POSITIVE power is in opposite direction of ratchet,
        // NEGATIVE power climbs
        +runCommand(FlywheelSubsystem) { FlywheelSubsystem.climbAtPower(0.8) }
                .withInterrupt { FlywheelSubsystem.shooterMaster.encoder.position > 77000.degrees }
                .withInterrupt { Controls.operatorXbox.getY(GenericHID.Hand.kRight).absoluteValue > 0.5 }
                .beforeStarting(Runnable { FlywheelSubsystem.shooterMaster.brakeMode = true })
                .andThen { FlywheelSubsystem.shooterMaster.setNeutral() }

        +PrintCommand("GOTEM COACH")

        +instantCommand(FlywheelSubsystem) { FlywheelSubsystem.engagePawl() }

        +OpenLoopClimbCommand().perpetually()
    }

private fun Command.andThen(vararg reqs: Subsystem, function: () -> Unit) = this.andThen(instantCommand(*reqs) { function() })
