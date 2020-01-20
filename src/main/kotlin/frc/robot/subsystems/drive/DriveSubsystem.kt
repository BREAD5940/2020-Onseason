package frc.robot.subsystems.drive

import com.kauailabs.navx.frc.AHRS
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.trajectory.Trajectory
import frc.robot.Constants
import frc.robot.subsystems.drive.swerve.Mk2SwerveModule
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.Job
import lib.mirror
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.debug.FalconDashboard
import org.ghrobotics.lib.mathematics.twodim.geometry.x_u
import org.ghrobotics.lib.mathematics.twodim.geometry.y_u
import org.ghrobotics.lib.mathematics.twodim.trajectory.mirror
import org.ghrobotics.lib.mathematics.units.Meter
import org.ghrobotics.lib.mathematics.units.SIUnit
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.inFeet
import org.ghrobotics.lib.mathematics.units.inches
import org.ghrobotics.lib.mathematics.units.nativeunit.SlopeNativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.ghrobotics.lib.utils.BooleanSource
import org.ghrobotics.lib.utils.asSource
import org.ghrobotics.lib.utils.launchFrequency

object DriveSubsystem : FalconSubsystem() {

    val gyro = AHRS(SPI.Port.kMXP).asSource()

    private val driveNativeUnitModel = SlopeNativeUnitModel(
            1.inches,
            (1.0 / (4.0 * Math.PI / 60.0 * 15.0 / 20.0 * 24.0 / 38.0 * 18.0)).nativeUnits)

    val kAzumithMotorOutputRange = -0.5..0.5

    val flModule = Mk2SwerveModule(2, 2, 142.degrees + 76.degrees, FalconMAX(
            CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless), driveNativeUnitModel),
            0.5, 0.0, 0.0001, kAzumithMotorOutputRange)

    val frModule = Mk2SwerveModule(4, 1, (87+4).degrees, FalconMAX(
            CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless), driveNativeUnitModel),
            0.5, 0.0, 0.0001, kAzumithMotorOutputRange)

    val blModule = Mk2SwerveModule(8, 0, 92.degrees - 25.degrees + 0.degrees, FalconMAX(
            CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless), driveNativeUnitModel),
            0.5, 0.0, 0.0001, kAzumithMotorOutputRange)

    val brModule = Mk2SwerveModule(6, 3, 42.degrees, FalconMAX(
            CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless), driveNativeUnitModel),
            0.5, 0.0, 0.0001, kAzumithMotorOutputRange)

    val modules = listOf(flModule, frModule, blModule, brModule)

    val feedForward = SimpleMotorFeedforward(
            (2.6),
            (0.0),
            (0.0)
    ).apply {
//        TODO("idk -- need to tune dis. Should be per module!")
    }

    val kinematics = Constants.kinematics

    internal val odometry = SwerveDriveOdometry(kinematics, gyro())

    private val stateLock = Object()
    val periodicIO = PeriodicIO()
        get() = synchronized(stateLock) { field }

    override fun lateInit() {

        flModule.driveMotor.canSparkMax.inverted = false
        frModule.driveMotor.canSparkMax.inverted = true
        blModule.driveMotor.canSparkMax.inverted = false
        brModule.driveMotor.canSparkMax.inverted = true
        modules.forEach { it.driveMotor.brakeMode = true }

        // set the default comand
        defaultCommand = HolomonicDriveCommand()
//        defaultCommand = RunCommand(Runnable{
//            periodicIO.output = SwerveDriveOutput.Nothing //SwerveDriveOutput.Percent(ChassisSpeeds(0.0, 0.0, 0.0))
//        }, this)

        // update localization f a s t
        this.kinematicsUpdateJob = GlobalScope.launchFrequency(200) {
            updateState()
            useState()
        }
    }

    fun setGyroAngle(angle: Rotation2d) {
        odometry.resetPosition(Pose2d(periodicIO.pose.translation, angle), gyro())
    }

    val currentSwerveModuleStates get() = listOf(flModule.state, frModule.state, blModule.state, brModule.state)

    val robotPosition get() = periodicIO.pose

    override fun periodic() {

        val job = kinematicsUpdateJob
        if ((job) != null) {
            if (!job.isActive) job.start()
        }

//        println("fl ${flModule.azimuthAngle().degrees.roundToInt()} bl ${blModule.azimuthAngle().degrees.roundToInt()} br ${brModule.azimuthAngle().degrees.roundToInt()}")

        FalconDashboard.robotHeading = robotPosition.rotation.radians
        FalconDashboard.robotX = robotPosition.translation.x_u.inFeet()
        FalconDashboard.robotY = robotPosition.translation.y_u.inFeet()
    }

    fun followTrajectory(trajectory: Trajectory, endHeading: Rotation2d, mirrored: Boolean = false) =
            SwerveTrajectoryFollowerCommand(if (mirrored) trajectory.mirror() else trajectory,
                    if (mirrored) endHeading.mirror() else endHeading)

    fun followTrajectory(trajectory: Trajectory, endHeading: Rotation2d, mirrored: BooleanSource) =
            SwerveTrajectoryFollowerCommand(trajectory, endHeading, mirrored)

    fun characterize() = SwerveCharacterizationCommand()

    override fun setNeutral() {
        periodicIO.output = SwerveDriveOutput.Nothing
    }

    var kinematicsUpdateJob: Job? = null
    private fun updateState() {
        modules.forEach { it.updateState() }

        // Update odometry
        val states = listOf(
                flModule.state,
                frModule.state,
                brModule.state,
                blModule.state)

        periodicIO.pose = odometry.update(gyro(), states[0], states[1], states[2], states[3])
        periodicIO.speed = kinematics.toChassisSpeeds(states[0], states[1], states[2], states[3])
    }

    fun useState() {
        // switch over our wanted state
        // and set module positions/outputs accordingly
        when (val output = periodicIO.output) {
            is SwerveDriveOutput.Nothing -> {
//                modules.forEach { it.output = Mk2SwerveModule.Output.Nothing }
                val states = currentSwerveModuleStates
                modules.forEachIndexed { i, module -> module.output = Mk2SwerveModule.Output.Percent(0.0, states[i].angle) }
            }
            is SwerveDriveOutput.Percent -> {
                // normalize wheel speeds
                val states = kinematics.toSwerveModuleStates(output.chassisSpeed, output.centerOfRotation)
                SwerveDriveKinematics.normalizeWheelSpeeds(states, 1.0)

                flModule.output = Mk2SwerveModule.Output.Percent(
                        states[0].speedMetersPerSecond,
                        states[0].angle
                )
                frModule.output = Mk2SwerveModule.Output.Percent(
                        states[1].speedMetersPerSecond,
                        states[1].angle
                )
                brModule.output = Mk2SwerveModule.Output.Percent(
                        states[2].speedMetersPerSecond,
                        states[2].angle
                )
                blModule.output = Mk2SwerveModule.Output.Percent(
                        states[3].speedMetersPerSecond,
                        states[3].angle
                )
            }
            is SwerveDriveOutput.Velocity -> {
                val states = kinematics.toSwerveModuleStates(output.chassisSpeed)
                modules.forEachIndexed { index, module ->
                    module.output = Mk2SwerveModule.Output.Velocity(SIUnit(states[index].speedMetersPerSecond), states[index].angle)
                }
            }
            is SwerveDriveOutput.KinematicsVoltage -> {
                flModule.output = Mk2SwerveModule.Output.Voltage(
                        SIUnit(output.speeds[0].speedMetersPerSecond),
                        output.speeds[0].angle
                )
                frModule.output = Mk2SwerveModule.Output.Voltage(
                        SIUnit(output.speeds[1].speedMetersPerSecond),
                        output.speeds[1].angle
                )
                brModule.output = Mk2SwerveModule.Output.Voltage(
                        SIUnit(output.speeds[2].speedMetersPerSecond),
                        output.speeds[2].angle
                )
                blModule.output = Mk2SwerveModule.Output.Voltage(
                        SIUnit(output.speeds[3].speedMetersPerSecond),
                        output.speeds[3].angle
                )
            }
            is SwerveDriveOutput.KinematicsVelocity -> {
                flModule.output = Mk2SwerveModule.Output.Velocity(
                        SIUnit(output.speeds[0].speedMetersPerSecond),
                        output.speeds[0].angle
                )
                frModule.output = Mk2SwerveModule.Output.Velocity(
                        SIUnit(output.speeds[1].speedMetersPerSecond),
                        output.speeds[1].angle
                )
                brModule.output = Mk2SwerveModule.Output.Velocity(
                        SIUnit(output.speeds[2].speedMetersPerSecond),
                        output.speeds[2].angle
                )
                blModule.output = Mk2SwerveModule.Output.Velocity(
                        SIUnit(output.speeds[3].speedMetersPerSecond),
                        output.speeds[3].angle
                )
            }
            is SwerveDriveOutput.TrajectoryTrackerOutput -> {
                flModule.output = output.flState
                frModule.output = output.frState
                brModule.output = output.brState
                blModule.output = output.blState
            }
        }

        modules.forEach { it.useState() }
    }

    class PeriodicIO {
        var pose = Pose2d()
        var speed = ChassisSpeeds()
        var output: SwerveDriveOutput = SwerveDriveOutput.Nothing
    }
}

sealed class SwerveDriveOutput {
    object Nothing : SwerveDriveOutput()

    class Percent(
        val chassisSpeed: ChassisSpeeds,
        val centerOfRotation: Translation2d = Translation2d()
    ) : SwerveDriveOutput()

    class Velocity(
        val chassisSpeed: ChassisSpeeds
    ) : SwerveDriveOutput()

    class KinematicsVelocity(
        val speeds: List<SwerveModuleState>
    ) : SwerveDriveOutput()

    class KinematicsVoltage(
        val speeds: List<SwerveModuleState>
    ) : SwerveDriveOutput()

    class TrajectoryTrackerOutput(
        val flState: Mk2SwerveModule.Output.Velocity,
        val frState: Mk2SwerveModule.Output.Velocity,
        val blState: Mk2SwerveModule.Output.Velocity,
        val brState: Mk2SwerveModule.Output.Velocity
    ) : SwerveDriveOutput() {
        constructor() : this (
                Mk2SwerveModule.Output.Velocity(),
                Mk2SwerveModule.Output.Velocity(),
                Mk2SwerveModule.Output.Velocity(),
                Mk2SwerveModule.Output.Velocity()
        )
    }
}
