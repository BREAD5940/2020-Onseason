package frc.robot.subsystems.shooter

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.Timer
import kotlin.math.abs
import org.ghrobotics.lib.commands.FalconCommand

class ShooterCharacterizationCommand : FalconCommand(FlywheelSubsystem) {

    var priorAutospeed = 0.0
    var autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed")
    var telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry")

    var numberArray = arrayOf<Number>(6)

    override fun initialize() {
        priorAutospeed = 0.0
    }

    override fun execute() {
        // Retrieve values to send back before telling the motors to do something
        // Retrieve values to send back before telling the motors to do something
        val now = Timer.getFPGATimestamp()

        val position: Double = FlywheelSubsystem.throughBoreEncoder.distance
        val rate: Double = FlywheelSubsystem.throughBoreEncoder.rate

        val battery = RobotController.getBatteryVoltage()
        val motorVolts: Double = battery * abs(priorAutospeed)

        // Retrieve the commanded speed from NetworkTables
        val autospeed: Double = autoSpeedEntry.getDouble(0.0)
        priorAutospeed = autospeed

        // command motors to do things
        FlywheelSubsystem.shootAtPower(autospeed)

        // send telemetry data array back to NT
        /*numberArray.get(0) = now
        numberArray.get(1) = battery
        numberArray.get(2) = autospeed
        numberArray.get(3) = motorVolts
        numberArray.get(4) = position
        numberArray.get(5) = rate*/
        numberArray = arrayOf(now, battery, autospeed, motorVolts, position, rate)
        telemetryEntry.setNumberArray(numberArray)
    }
}
