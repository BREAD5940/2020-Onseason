package frc.robot.subsystems.drive

import com.kauailabs.navx.frc.AHRS
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.Compressor
import edu.wpi.first.wpilibj.SPI
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.trajectory.Trajectory
import edu.wpi.first.wpilibj.util.Units
import edu.wpi.first.wpiutil.math.VecBuilder
import frc.robot.Constants
import frc.robot.subsystems.drive.DriveSubsystem.feedForward
import frc.robot.subsystems.drive.swerve.Mk2SwerveModule
import frc.robot.subsystems.vision.VisionSubsystem
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.Job
import lib.Logger
import lib.asSparkMax
import lib.mirror
import org.ghrobotics.lib.commands.FalconSubsystem
import org.ghrobotics.lib.debug.FalconDashboard
import org.ghrobotics.lib.localization.TimePoseInterpolatableBuffer
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.x_u
import org.ghrobotics.lib.mathematics.twodim.geometry.y_u
import org.ghrobotics.lib.mathematics.twodim.trajectory.mirror
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.*
import org.ghrobotics.lib.mathematics.units.nativeunit.SlopeNativeUnitModel
import org.ghrobotics.lib.mathematics.units.nativeunit.nativeUnits
import org.ghrobotics.lib.motors.rev.FalconMAX
import org.ghrobotics.lib.utils.BooleanSource
import org.ghrobotics.lib.utils.Source
import org.ghrobotics.lib.utils.launchFrequency
import org.ghrobotics.lib.utils.map
import java.lang.RuntimeException

object DriveSubsystem : FalconSubsystem() {

    private val navX = AHRS(SPI.Port.kMXP)
    private val robotHeadingSource = { -Rotation2d.fromDegrees(navX.fusedHeading.toDouble()) }

    val compressor = Compressor(8).apply { clearAllPCMStickyFaults() }

    /**
     * This native unit model converts sensor ticks (in this case, rotations of the NEO) to a SIUnit<Meter>.
     * Basically we tell it how many rotations of the neo per inch of travel.
     */
    private val driveNativeUnitModel = SlopeNativeUnitModel(
            1.inches,
            (1.0 / (4.0 * Math.PI / 60.0 * 15.0 / 20.0 * 24.0 / 38.0 * 18.0)).nativeUnits)

    // max duty cycle to demand of swerve module motors
    private val kAzimuthMotorOutputRange = -0.5..0.5

    val brModule = Mk2SwerveModule(4, 3, 254.degrees - 254.degrees, 0.5,
            0.0, 0.0001, FalconMAX(
                    CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless), driveNativeUnitModel), kAzimuthMotorOutputRange, "br")

    val blModule = Mk2SwerveModule(6, 2, 273.degrees - 164.degrees, 0.5,
            0.0, 0.0001, FalconMAX(
                    CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless), driveNativeUnitModel), kAzimuthMotorOutputRange, "bl")

    val frModule = Mk2SwerveModule(2, 1, -18.degrees, 0.5,
            0.0, 0.0001, FalconMAX(
                    CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless), driveNativeUnitModel), kAzimuthMotorOutputRange, "fr")

    val flModule = Mk2SwerveModule(8, 0, -24.degrees + 72.degrees - 180.degrees, 0.5,
            0.0, 0.0001, FalconMAX(
                    CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless), driveNativeUnitModel), kAzimuthMotorOutputRange, "fl")

    private val modules = listOf(flModule, frModule, blModule, brModule)

    internal val field = Field2d()

    /**
     * Feedforward. Used for trajectory tracking
     */
    val feedForward = SimpleMotorFeedforward(
            (0.15), // ks
            (2.9), // kv, volts per meter per second
            (0.3) // ka
    )

    val kinematics = Constants.kinematics

    private val odometry = SwerveDriveOdometry(kinematics, robotHeadingSource()).apply {
        resetPosition(Pose2d(30.feet, 11.feet, 180.degrees), robotHeadingSource()) // curst so i can be lazy
    }

    private val estimator = SwerveDrivePoseEstimator(robotHeadingSource(), Pose2d(30.feet, 11.feet, 180.degrees),
        kinematics,
        VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5.0)),
        VecBuilder.fill(Units.degreesToRadians(1.0)),
        VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10.0)),
        1.0 / 200.0)

    fun resetPosition(pose: Pose2d) {
        odometry.resetPosition(pose, robotHeadingSource())
        estimator.resetPosition(pose, robotHeadingSource())
    }

    fun addVisionPose(fieldToRobot: Pose2d, time: SIUnit<Second>) {
//        try {
//            estimator.addVisionMeasurement(fieldToRobot, time.inSeconds())
//        } catch (e: RuntimeException) {
//            e.printStackTrace()
//        }

        field.getObject("vision pose").pose = fieldToRobot
    }

    val logger = Logger("DriveSubsystem")

    private val stateLock = Object()
    val periodicIO = PeriodicIO()
        get() = synchronized(stateLock) { field }

    /**
     * Buffer for storing the pose over a span of time. This is useful for
     * Vision and latency compensation.
     */
    val poseBuffer = TimePoseInterpolatableBuffer()

    override fun lateInit() {

        flModule.driveMotor.asSparkMax()?.canSparkMax?.inverted = false
        frModule.driveMotor.canSparkMax.inverted = true
        blModule.driveMotor.canSparkMax.inverted = false
        brModule.driveMotor.canSparkMax.inverted = true
        modules.forEach { it.driveMotor.brakeMode = true }
        modules.forEach { it.azimuthMotor.brakeMode = false }

        // set the default command
        defaultCommand = HolomonicDriveCommand()

        // update localization at 200hz
        this.kinematicsUpdateJob = GlobalScope.launchFrequency(200) {
            updateState()
            useState()
        }

        // write CSV header
        // logger.log("flAngle, flAzimuthVolt, flDriveAngle, flDriveVolt, " +
        //        "frAngle, frAzimuthVolt, frDriveAngle, frDriveVolt, " +
        //        "blAngle, blAzimuthVolt, blDriveAngle, blDriveVolt, " +
        //        "brAngle, brAzimuthVolt, brDriveAngle, brDriveVolt,")
        logger.log("time, x, y, heading (deg)")


        SmartDashboard.putData("Field", field)

        compressor.start()
    }

    fun setGyroAngle(angle: Rotation2d) {
        resetPosition(Pose2d(robotPosition.translation, angle))
    }

    val currentSwerveModuleStates get() = listOf(flModule.currentState, frModule.currentState, blModule.currentState, brModule.currentState)

    var robotPosition
        get() = periodicIO.pose
        set(value) {
            resetPosition(value)
        }

    override fun periodic() {

        val job = kinematicsUpdateJob
        if ((job) != null) {
            if (!job.isActive) job.start()
        }

        // update FalconDashboard
        FalconDashboard.robotHeading = robotPosition.rotation.radians
        FalconDashboard.robotX = robotPosition.translation.x_u.inFeet()
        FalconDashboard.robotY = robotPosition.translation.y_u.inFeet()

        poseBuffer[Timer.getFPGATimestamp().seconds] = robotPosition

        // Update 2d field pose
        field.getObject("close goal").pose = VisionSubsystem.closePowerPort
        field.getObject("far goal").pose = VisionSubsystem.farPowerPort
        field.robotPose = robotPosition
//        field.getObject("pose estimator").pose = estimator.estimatedPosition
//        field.getObject("odometry").pose = robotPosition
    }

    // Trajectory following and other utility methods

    fun followTrajectory(trajectory: Trajectory, endHeading: Rotation2d, mirrored: Boolean = false) =
            SwerveTrajectoryFollowerCommand(if (mirrored) trajectory.mirror() else trajectory,
                    if (mirrored) endHeading.mirror() else endHeading)

    fun followTrajectory(trajectory: Trajectory, endHeading: Source<Rotation2d>) =
            SwerveTrajectoryFollowerCommand({ trajectory }, endHeading)

    fun followTrajectory2(trajectory: Trajectory, endHeading: Source<SIUnit<Radian>>) =
            SwerveTrajectoryFollowerCommand({ trajectory }, endHeading.map { it.toRotation2d() })

    fun followTrajectory(trajectory: Trajectory, endHeading: Source<Rotation2d>, mirrored: BooleanSource = { false }) =
            SwerveTrajectoryFollowerCommand(mirrored.map(trajectory.mirror(), trajectory),
                    mirrored.map { if (mirrored()) endHeading().mirror() else endHeading() })

    fun followTrajectory(trajectory: Trajectory, endHeading: Rotation2d, mirrored: BooleanSource) =
            SwerveTrajectoryFollowerCommand(trajectory, endHeading, mirrored)

    fun characterize() = SwerveCharacterizationCommand()

    override fun setNeutral() {
        periodicIO.output = SwerveDriveOutput.Nothing
    }

    var kinematicsUpdateJob: Job? = null

    /**
     * updateState updates the robot's estimated state. Right now just updates odometry.
     */
    private fun updateState() {
        modules.forEach { it.updateState() }

        // Update odometry
        val states = listOf(
                flModule.currentState,
                frModule.currentState,
                brModule.currentState,
                blModule.currentState)

        periodicIO.pose = odometry.update(robotHeadingSource(), states[0], states[1], states[2], states[3])
        periodicIO.speed = kinematics.toChassisSpeeds(states[0], states[1], states[2], states[3])

        // Update pose estimator
        estimator.update(robotHeadingSource(), states[0], states[1], states[2], states[3])

        // val output = modules.map { "${it.azimuthAngle().degrees}, ${it.azimuthMotor.voltageOutput.value}, ${it.driveMotor.encoder.velocity.value}, ${it.driveMotor.voltageOutput.value}" }
        // logger.log("${output[0]}, ${output[1]}, ${output[2]}, ${output[3]}")

        logger.log(Timer.getFPGATimestamp(), robotPosition.x, robotPosition.y, robotPosition.rotation.degrees)
    }

    /**
     * useStates calculates demands for each of our modules and sends it to them, then calls [Mk2SwerveModule.useState] on them.
     */
    fun useState() {
        // switch over our wanted state
        // and set module positions/outputs accordingly
        when (val output = periodicIO.output) {
            is SwerveDriveOutput.Nothing -> {
//                modules.forEach { it.output = Mk2SwerveModule.Output.Nothing }
                val states = currentSwerveModuleStates
                modules.forEachIndexed { i, module -> module.currentOutput = Mk2SwerveModule.Output.DutyCycle(0.0, states[i].angle) }
            }
            is SwerveDriveOutput.Percent -> {
                // normalize wheel speeds
                val states = kinematics.toSwerveModuleStates(output.chassisSpeed, output.centerOfRotation)
                SwerveDriveKinematics.normalizeWheelSpeeds(states, 1.0)

//                println("chassis speeds: ${output.chassisSpeed} \ncor ${output.centerOfRotation}\nangles:\n" + states.map { it.angle.degrees }
//                + "\n wheel speeds:\n"+states.map { it.speedMetersPerSecond })

                flModule.currentOutput = Mk2SwerveModule.Output.DutyCycle(
                        states[0].speedMetersPerSecond,
                        states[0].angle
                )
                frModule.currentOutput = Mk2SwerveModule.Output.DutyCycle(
                        states[1].speedMetersPerSecond,
                        states[1].angle
                )
                brModule.currentOutput = Mk2SwerveModule.Output.DutyCycle(
                        states[2].speedMetersPerSecond,
                        states[2].angle
                )
                blModule.currentOutput = Mk2SwerveModule.Output.DutyCycle(
                        states[3].speedMetersPerSecond,
                        states[3].angle
                )
            }
            is SwerveDriveOutput.Velocity -> {
                val states = kinematics.toSwerveModuleStates(output.chassisSpeed)
                modules.forEachIndexed { index, module ->
                    module.currentOutput = Mk2SwerveModule.Output.Velocity(SIUnit(states[index].speedMetersPerSecond), states[index].angle)
                }
            }
            is SwerveDriveOutput.KinematicsVoltage -> {
                flModule.currentOutput = Mk2SwerveModule.Output.Voltage(
                        SIUnit(output.speeds[0].speedMetersPerSecond),
                        output.speeds[0].angle
                )
                frModule.currentOutput = Mk2SwerveModule.Output.Voltage(
                        SIUnit(output.speeds[1].speedMetersPerSecond),
                        output.speeds[1].angle
                )
                brModule.currentOutput = Mk2SwerveModule.Output.Voltage(
                        SIUnit(output.speeds[2].speedMetersPerSecond),
                        output.speeds[2].angle
                )
                blModule.currentOutput = Mk2SwerveModule.Output.Voltage(
                        SIUnit(output.speeds[3].speedMetersPerSecond),
                        output.speeds[3].angle
                )
            }
            is SwerveDriveOutput.KinematicsVelocity -> {
                flModule.currentOutput = Mk2SwerveModule.Output.Velocity(
                        SIUnit(output.speeds[0].speedMetersPerSecond),
                        output.speeds[0].angle
                )
                frModule.currentOutput = Mk2SwerveModule.Output.Velocity(
                        SIUnit(output.speeds[1].speedMetersPerSecond),
                        output.speeds[1].angle
                )
                brModule.currentOutput = Mk2SwerveModule.Output.Velocity(
                        SIUnit(output.speeds[2].speedMetersPerSecond),
                        output.speeds[2].angle
                )
                blModule.currentOutput = Mk2SwerveModule.Output.Velocity(
                        SIUnit(output.speeds[3].speedMetersPerSecond),
                        output.speeds[3].angle
                )
            }
            is SwerveDriveOutput.TrajectoryTrackerOutput -> {
                flModule.currentOutput = output.flState
                frModule.currentOutput = output.frState
                brModule.currentOutput = output.brState
                blModule.currentOutput = output.blState
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
        constructor(states: Array<SwerveModuleState>) : this (
                Mk2SwerveModule.Output.Velocity(
                        states[0].speedMetersPerSecond.meters.velocity, states[0].angle,
                        feedForward.calculate(states[0].speedMetersPerSecond).volts),
                Mk2SwerveModule.Output.Velocity(
                        states[1].speedMetersPerSecond.meters.velocity, states[1].angle,
                        feedForward.calculate(states[1].speedMetersPerSecond).volts),
                Mk2SwerveModule.Output.Velocity(
                        states[2].speedMetersPerSecond.meters.velocity, states[2].angle,
                        feedForward.calculate(states[2].speedMetersPerSecond).volts),
                Mk2SwerveModule.Output.Velocity(
                        states[3].speedMetersPerSecond.meters.velocity, states[3].angle,
                        feedForward.calculate(states[3].speedMetersPerSecond).volts)
        )
        //  DriveSubsystem.periodicIO.output = SwerveDriveOutput.KineamaticsVoltage(......blah.....)
    }
}
