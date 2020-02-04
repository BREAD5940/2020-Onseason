// package frc.robot.subsystems.drive
//
// import edu.wpi.first.wpilibj.Timer
// import edu.wpi.first.wpilibj.controller.PIDController
// import edu.wpi.first.wpilibj.geometry.Rotation2d
// import edu.wpi.first.wpilibj.geometry.Translation2d
// import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
// import edu.wpi.first.wpilibj.kinematics.SwerveModuleState
// import edu.wpi.first.wpilibj.trajectory.Trajectory
// import frc.robot.subsystems.drive.swerve.SwerveTrajectoryController
// import lib.mirror
// import org.ghrobotics.lib.commands.FalconCommand
// import org.ghrobotics.lib.debug.FalconDashboard
// import org.ghrobotics.lib.mathematics.twodim.geometry.x_u
// import org.ghrobotics.lib.mathematics.twodim.geometry.y_u
// import org.ghrobotics.lib.mathematics.twodim.trajectory.mirror
// import org.ghrobotics.lib.mathematics.units.inFeet
// import org.ghrobotics.lib.utils.BooleanSource
// import org.ghrobotics.lib.utils.Source
// import org.ghrobotics.lib.utils.map
//
// /**
// * Follow a trajectory
// * @param trajectorySupplier the trajectory to follow, paired with the end heading of the drivetrain
// */
// class SwerveTrajectoryFollowerCommand(
//        val trajectorySupplier: Source<Trajectory>,
//        val headingSupplier: Source<Rotation2d>
// ) : FalconCommand(DriveSubsystem) {
//
//    constructor(trajectory: Trajectory, targetHeading: Rotation2d, pathMirrored: BooleanSource) : this(
//            pathMirrored.map(trajectory.mirror(), trajectory),
//            pathMirrored.map(targetHeading.mirror(), targetHeading)
//    )
//
//    constructor(trajectory: Trajectory, targetHeading: Rotation2d) : this(
//            { trajectory }, { targetHeading }
//    )
//
//    private val forwardController = PIDController(1.0, 0.0, 0.0) // x meters per second per meter of error
//    private val strafeController = PIDController(1.0, 0.0, 0.0)
//    private val rotationController = PIDController(0.5, 0.0, 0.0) // rad per sec per radian of error
//
//
// //    private var prevStates = listOf<SwerveModuleState>()
//    private lateinit var trajectory: Trajectory
//    private lateinit var targetHeading: Rotation2d
//    private val timer = Timer()
//
//    private val controller = SwerveTrajectoryController(DriveSubsystem.kinematics, DriveSubsystem.feedForward)
//
//    override fun isFinished() = timer.hasPeriodPassed(trajectory.totalTimeSeconds)
//
//    override fun end(interrupted: Boolean) {
//        FalconDashboard.isFollowingPath = false
//    }
//
//    override fun initialize() {
//        trajectory = trajectorySupplier()
//        targetHeading = headingSupplier()
//
//        FalconDashboard.isFollowingPath = true
//
//        timer.reset()
//        timer.start()
// //        prevStates = listOf(SwerveModuleState(), SwerveModuleState(), SwerveModuleState(), SwerveModuleState())
//    }
//
//    override fun execute() {
//
//        // update the controller
//        val time = timer.get()
//        val state = trajectory.sample(time)
//        val robotPose = DriveSubsystem.periodicIO.pose
// //        DriveSubsystem.periodicIO.output = controller.calculate(time, state, targetHeading, DriveSubsystem.periodicIO.pose)
//
//        val targetTranslation = state.poseMeters.translation
//        val robotTranslation = robotPose.translation
//        val eX = (targetTranslation.x - robotTranslation.x)
//        val eY = (targetTranslation.y - robotTranslation.y)
//
//        val targetVelocity = state.velocityMetersPerSecond
//        val targetHeading = state.poseMeters.rotation
//
//        val eHeading = targetHeading - robotPose.rotation
//
//        val feedbackOutput = ChassisSpeeds.fromFieldRelativeSpeeds(
//                forwardController.calculate(currentPose.translation.x, dt) + velocity.x,
//                strafeController.calculate(currentPose.translation.y, dt) + velocity.y,
//                rotationController.calculate(currentPose.rotation.radians, dt),
//                currentPose.rotation
//        )
//
//
//
//
//        FalconDashboard.pathX = state.poseMeters.translation.x_u.inFeet()
//        FalconDashboard.pathY = state.poseMeters.translation.y_u.inFeet()
//        FalconDashboard.pathHeading = state.poseMeters.rotation.radians
//    }
// }
//
// //fun Rotation2d.toTranslation2d() = Translation2d(this.cos, this.sin)
