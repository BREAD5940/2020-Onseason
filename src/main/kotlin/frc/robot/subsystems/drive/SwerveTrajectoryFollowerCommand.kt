package frc.robot.subsystems.drive

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.trajectory.Trajectory
import frc.robot.subsystems.drive.swerve.SwerveTrajectoryController
import lib.mirror
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.debug.FalconDashboard
import org.ghrobotics.lib.mathematics.twodim.geometry.x_u
import org.ghrobotics.lib.mathematics.twodim.geometry.y_u
import org.ghrobotics.lib.mathematics.twodim.trajectory.mirror
import org.ghrobotics.lib.mathematics.units.inFeet
import org.ghrobotics.lib.utils.BooleanSource
import org.ghrobotics.lib.utils.Source
import org.ghrobotics.lib.utils.map
import kotlin.math.abs

/**
 * Follow a trajectory
 * @param trajectorySupplier the trajectory to follow, paired with the end heading of the drivetrain
 */
class SwerveTrajectoryFollowerCommand(
    val trajectorySupplier: Source<Trajectory>,
    val headingSupplier: Source<Rotation2d>
) : FalconCommand(DriveSubsystem) {

    constructor(trajectory: Trajectory, targetHeading: Rotation2d, pathMirrored: BooleanSource) : this(
            pathMirrored.map(trajectory.mirror(), trajectory),
            pathMirrored.map(targetHeading.mirror(), targetHeading)
    )

    constructor(trajectory: Trajectory, targetHeading: Rotation2d) : this(
            { trajectory }, { targetHeading }
    )

    private var prevStates = listOf<SwerveModuleState>()
    private lateinit var trajectory: Trajectory
    private lateinit var targetHeading: Rotation2d
    private val timer = Timer()

    private val controller = SwerveTrajectoryController(DriveSubsystem.kinematics, DriveSubsystem.feedForward)

    override fun isFinished() = timer.hasPeriodPassed(trajectory.totalTimeSeconds);

    override fun end(interrupted: Boolean) {
        FalconDashboard.isFollowingPath = false
        DriveSubsystem.setNeutral()
    }

    override fun initialize() {
        trajectory = trajectorySupplier()

        FalconDashboard.isFollowingPath = true

        timer.reset()
        timer.start()
        prevStates = listOf(SwerveModuleState(), SwerveModuleState(), SwerveModuleState(), SwerveModuleState())

        controller.reset(DriveSubsystem.robotPosition.rotation)
    }

    override fun execute() {

        // update target heading
        targetHeading = headingSupplier()
        println(targetHeading.degrees)

        // update the controller
        val time = timer.get()
        val state = trajectory.sample(time)
        DriveSubsystem.periodicIO.output = controller.calculate(time, state, targetHeading, DriveSubsystem.periodicIO.pose)

        FalconDashboard.pathX = state.poseMeters.translation.x_u.inFeet()
        FalconDashboard.pathY = state.poseMeters.translation.y_u.inFeet()
        FalconDashboard.pathHeading = state.poseMeters.rotation.radians
    }
}

fun Rotation2d.toTranslation2d() = Translation2d(this.cos, this.sin)
