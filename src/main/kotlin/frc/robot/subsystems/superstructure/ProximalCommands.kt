package frc.robot.subsystems.superstructure

import edu.wpi.first.wpilibj.controller.ArmFeedforward
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.ProfiledPIDController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.*
import kotlin.math.roundToInt

class ClosedLoopProximalMove(val targetPosition: SIUnit<Radian> = 0.radians): FalconCommand(Proximal) {

    private val controller = PIDController(2.0, 0.0, 0.0)

    companion object {
        private val feedforward = ArmFeedforward(0.3, 0.3, 3.9, 0.08)
        private val constraints = TrapezoidProfile.Constraints(90.degrees.inRadians(), 200.degrees.inRadians())
    }

    lateinit var lastSetpoint: TrapezoidProfile.State

    val goal = TrapezoidProfile.State(targetPosition.value, 0.0)

    init {
        SmartDashboard.putData("pid", controller)
    }

    override fun initialize() {
        controller.reset()
//        controller.setGoal(targetPosition.inRadians())
        lastSetpoint = TrapezoidProfile.State(Proximal.position.value, Proximal.velocity.value)
    }

    override fun execute() {
        val position = Proximal.position

        val calculated = TrapezoidProfile(constraints, goal, lastSetpoint).calculate(0.020)
        lastSetpoint = calculated

        val fb = controller.calculate(position.inRadians(), calculated.position)

        println("pos ${position.inDegrees().roundToInt()} goal ${targetPosition.inDegrees().roundToInt()} setpoint ${calculated.position.radians.inDegrees().roundToInt()} fb ${fb.roundToInt()}")

        Proximal.voltageOutput = (fb
            + feedforward.calculate(Proximal.position.inRadians(), calculated.velocity)).volts
    }

    override fun isFinished() = false// controller.atGoal()

}