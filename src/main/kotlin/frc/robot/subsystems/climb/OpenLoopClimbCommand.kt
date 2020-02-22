package frc.robot.subsystems.climb

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.Controls
import frc.robot.subsystems.shooter.FlywheelSubsystem
import lib.instantCommand
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.utils.withDeadband
import org.ghrobotics.lib.wrappers.hid.getY

class OpenLoopClimbCommand : FalconCommand(FlywheelSubsystem) {

    override fun initialize() {
        FlywheelSubsystem.wantsShootMode = false
        FlywheelSubsystem.setClimberArmExtension(true)
        FlywheelSubsystem.disengagePawl()
    }

    override fun execute() {
        FlywheelSubsystem.climbAtPower(speedSource())
    }

    override fun end(interrupted: Boolean) {
        FlywheelSubsystem.engagePawl()
        FlywheelSubsystem.disableMotors()
    }

    companion object {
        val speedSource by lazy { Controls.operatorFalconXbox.getY(GenericHID.Hand.kRight).withDeadband(0.15) }
    }
}

val openLoopClimbCommandGroup
    get() = sequential {
        +instantCommand(BumperGrabberSubsystem) {
            BumperGrabberSubsystem.wantsExtended = true
        }
        +WaitCommand(1.0)
        +instantCommand(FlywheelSubsystem) {
            FlywheelSubsystem.wantsShootMode = false
            FlywheelSubsystem.setClimberArmExtension(true)
            FlywheelSubsystem.disengagePawl()
        }
        +WaitCommand(2.0)
                // negative power is in opposite direction of ratchet,
                // positive power climbs
                .alongWith(instantCommand(FlywheelSubsystem) { FlywheelSubsystem.climbAtPower(-0.15) }) // to engage dog
        +OpenLoopClimbCommand()
    }