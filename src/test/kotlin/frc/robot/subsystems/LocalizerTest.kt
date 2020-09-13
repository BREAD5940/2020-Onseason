// package frc.robot.subsystems
//
// import edu.wpi.first.hal.NotifierJNI
// import edu.wpi.first.wpilibj.Timer
// import edu.wpi.first.wpilibj.geometry.Pose2d
// import edu.wpi.first.wpilibj.geometry.Rotation2d
// import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
// import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig
// import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator
// import frc.robot.subsystems.drive.localization.DrivetrainLocalizer
// import java.util.*
// import kotlin.collections.ArrayList
// import org.ghrobotics.lib.mathematics.units.inMicroseconds
// import org.ghrobotics.lib.mathematics.units.seconds
// import org.junit.Test
// import org.knowm.xchart.XYChartBuilder
//
// class LocalizerTest {
//
//    @Test fun testLocalization() {
//
//        val trajectory = TrajectoryGenerator.generateTrajectory(
//                listOf(
//                        Pose2d(0.0, 0.0, Rotation2d()),
//                        Pose2d(5.0, 5.0, Rotation2d())
//                ), TrajectoryConfig(2.0, 2.0))
//
//        val xhatsX: ArrayList<Double> = ArrayList()
//        val measurementsX: ArrayList<Double> = ArrayList()
//        val xhatsY: ArrayList<Double> = ArrayList()
//        val measurementsY: ArrayList<Double> = ArrayList()
//
//        System.setOut(System.out)
//
//        var time = 0.0
//        val notifier = NotifierJNI.initializeNotifier()
//        val rand = Random()
//        while (time < trajectory.totalTimeSeconds) {
//            time += 0.005
//            NotifierJNI.updateNotifierAlarm(notifier, (Timer.getFPGATimestamp().seconds.inMicroseconds() + 0.005).toLong())
//            val state = trajectory.sample(time)
//
//            // update sometimes
//            var measuredPose: Pose2d? = null
//            if (rand.nextBoolean() && rand.nextBoolean()) {
//                val dev = 1.0 / 20.0
//                measuredPose = Pose2d(
//                        state.poseMeters.translation.x + rand.nextGaussian() * dev,
//                        state.poseMeters.translation.y + rand.nextGaussian() * dev,
//                        Rotation2d(state.poseMeters.rotation.radians + rand.nextGaussian() * dev)
//                )
//            }
//            val speeds = ChassisSpeeds(state.velocityMetersPerSecond * state.poseMeters.rotation.cos,
//                    state.velocityMetersPerSecond * state.poseMeters.rotation.sin,
//                    state.curvatureRadPerMeter * state.velocityMetersPerSecond)
//
//            if (measuredPose == null) {
//                DrivetrainLocalizer.update(speeds)
//            } else {
//                DrivetrainLocalizer.update(speeds, measuredPose, Timer.getFPGATimestamp().seconds - 0.2.seconds)
//            }
//
//            measuredPose?.translation?.x?.let { measurementsX.add(it) }
//            measuredPose?.translation?.y?.let { measurementsY.add(it) }
//            xhatsX.add(DrivetrainLocalizer.estimatedPosition.translation.x)
//            xhatsY.add(DrivetrainLocalizer.estimatedPosition.translation.y)
//
//            NotifierJNI.waitForNotifierAlarm(notifier)
//        }
//
//        val chart = XYChartBuilder().build()
//        chart.addSeries("Xhat, x/y", xhatsX, xhatsY)
//        if (measurementsX.size > 1) {
//            chart.addSeries("Measured position, x/y", measurementsX, measurementsY)
//        }
//
//        try {
// //            SwingWrapper(chart).displayChart()
// //            Thread.sleep(10000000)
//        } catch (ign: Exception) {
//        }
//    }
// }
