package frc.robot.subsystems.shooter

import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Constants
import frc.robot.Robot
import frc.robot.subsystems.drive.VisionDriveCommand
import frc.team4069.keigen.*
import kotlin.math.abs
import lib.Logger
import lib.inRpm
import lib.revolutionsPerMinute
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.*

/**
 * Set the flywheel to shoot at a speed specified by a supplier. This command will run
 * forever if [endAfterSpinup] is false, and will exit when the hood and flywheel
 * are spun up and at the correct angle if [endAfterSpinup] is true.
 */
class ShootCommand(private val parameterSupplier: () -> ShotParameter, private val endAfterSpinup: Boolean = false) : FalconCommand(FlywheelSubsystem, HoodSubsystem) {

    constructor(endAfterSpinup: Boolean = false) : this(
            fun(): ShotParameter {
                var pose = VisionDriveCommand.getTargetPose()
                if (pose == null) pose = lastPose
                lastPose = pose

                val distance = pose.translation.norm
                val table = Constants.distanceLookupTable5v
                var param = table.get(distance)
                if (param == null) {
                    param = lastParam
                }
                lastParam = param

                return param

//                distance = (Constants.distanceLookupTable5v.get((VisionDriveCommand.getTargetPose() ?: lastPose)
//                        .apply { lastPose = this }.translation.norm) ?: Constants.rightBelowGoalParameter5v)
            },
            endAfterSpinup)

//    constructor(endAfterSpinup: Boolean = false) : this(
//            { Constants.pitchLookupTable5v.get((
//                    if(VisionSubsystem.lifecam.isValid) VisionSubsystem.lifecam.pitch.degrees else 0.0))
//                    ?: ShotParameter.DefaultParameter }, endAfterSpinup)

    constructor(hoodAngle: SIUnit<Radian>, speed: SIUnit<Velocity<Radian>>, endAfterSpinup: Boolean = false) : this({ ShotParameter(hoodAngle, speed) }, endAfterSpinup)

    private val angleEntry: NetworkTableEntry = SmartDashboard.getEntry("hoodAngle")
    private val rpmEntry: NetworkTableEntry = SmartDashboard.getEntry("rpm")

    override fun initialize() {
        angleEntry.setDefaultDouble(45.0)
        rpmEntry.setDefaultDouble(0.0)

        ShooterController.reset()
        ShooterController.enable()
//        ShooterController.loop.xHat = vec(`1`).fill(FlywheelSubsystem.flywheelSpeed.value)

        logger.clearLog()
        logger.log("setpoint, measurement, xhat, voltage")
    }

    override fun execute() {
        var wantedParameter = parameterSupplier()
//        val wantedParameter = ShotParameter(angleEntry.getDouble(45.0).degrees, rpmEntry.getDouble(0.0).revolutionsPerMinute)

        if (Robot.debugMode) {
            wantedParameter = ShotParameter(angleEntry.getDouble(45.0).degrees, rpmEntry.getDouble(0.0).revolutionsPerMinute)
        }

        HoodSubsystem.wantedAngle = wantedParameter.hoodAngle + hoodAngleOffset

        // call periodically to recalculate feedback
//        FlywheelSubsystem.shootAtSpeed(wantedParameter.speed)

        ShooterController.setSpeed(wantedParameter.speed)
        ShooterController.update(FlywheelSubsystem.flywheelSpeed)
        val volts = ShooterController.nextU

        FlywheelSubsystem.shootAtVoltage(volts)

        SmartDashboard.putNumber("distanceToGoal", (VisionDriveCommand.getTargetPose() ?: Pose2d()).translation.norm)
        SmartDashboard.putString("better parameter", (Constants.distanceLookupTable5v.get((VisionDriveCommand.getTargetPose()
                ?: Pose2d()).translation.norm) ?: Constants.rightBelowGoalParameter5v).toString())

//        Constants.distanceLookupTable5v.get((VisionDriveCommand.getTargetPose() ?: Pose2d()).translation.norm) ?: Constants.rightBelowGoalParameter5v

//        setpoint, measurement, xhat, voltage
        logger.log(wantedParameter.speed.inRpm(), FlywheelSubsystem.flywheelSpeed.inRpm(), ShooterController.xHat.inRpm(), volts.value)
    }

    private fun isOnTarget(): Boolean {
        val wantedParameter = if (Robot.debugMode)
            ShotParameter(angleEntry.getDouble(45.0).degrees, rpmEntry.getDouble(0.0).revolutionsPerMinute)
        else
            parameterSupplier()

        return abs(wantedParameter.speed.inRpm() - FlywheelSubsystem.flywheelSpeed.inRpm()) < 100 &&
                abs(HoodSubsystem.wantedAngle.inDegrees() - (wantedParameter.hoodAngle + hoodAngleOffset).inDegrees()) < 1.0 // TODO hood angle comparison compares wantedangle to wantedangle, should compare wanted to current
    }

    override fun isFinished(): Boolean {
        return endAfterSpinup && isOnTarget()
    }

    override fun end(interrupted: Boolean) {
        FlywheelSubsystem.setNeutral()

        ShooterController.disable()
    }

    companion object {
        var lastPose = Pose2d()
        var lastParam = ShotParameter.defaultParameter
        val logger = Logger("Shooter")

        var hoodAngleOffset = 0.degrees
    }
}
