package frc.robot.subsystems.shooter

import edu.wpi.first.wpilibj.Timer
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.amps
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.velocity
import org.ghrobotics.lib.mathematics.units.derived.volts

class ZeroCommand : FalconCommand(HoodSubsystem) {

    val timer = Timer()

    override fun initialize() {
        super.initialize()
        HoodSubsystem.hoodMotor.smartCurrentLimit = 3.amps
        HoodSubsystem.hoodMotor.canSparkMax.setSecondaryCurrentLimit(5.0)
        HoodSubsystem.wantedState = HoodSubsystem.State.Homing
        HoodSubsystem.hoodMotor.setVoltage(-3.volts)

        timer.reset()
        timer.start()
    }

    override fun isFinished(): Boolean {
        return HoodSubsystem.hoodMotor.encoder.velocity.absoluteValue < 10.degrees.velocity &&
                timer.get() > 0.25
    }

    override fun end(interrupted: Boolean) {
        super.end(interrupted)
        HoodSubsystem.wantedState = HoodSubsystem.State.Position
        HoodSubsystem.hoodMotor.encoder.resetPosition(17.degrees)
        HoodSubsystem.hoodMotor.smartCurrentLimit = 5.amps
        HoodSubsystem.hoodMotor.canSparkMax.setSecondaryCurrentLimit(10.0)
    }
}
