// package frc.robot.subsystems
//
// import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator
// import edu.wpi.first.wpilibj.geometry.Pose2d
// import edu.wpi.first.wpilibj.geometry.Rotation2d
// import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry
// import edu.wpi.first.wpilibj.system.LinearSystem
// import edu.wpi.first.wpiutil.math.MatrixUtils
// import frc.team4069.keigen.*
// import org.ghrobotics.lib.mathematics.units.derived.degrees
// import org.ghrobotics.lib.mathematics.units.derived.inRadians
// import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
// import org.ghrobotics.lib.mathematics.units.inMeters
// import org.ghrobotics.lib.mathematics.units.inches
// import org.junit.Test
// import org.knowm.xchart.SwingWrapper
// import org.knowm.xchart.XYChartBuilder
// import java.io.BufferedReader
// import java.io.File
// import java.io.FileReader
// import kotlin.math.PI
//
// class StateSpaceMemes {
//
//    @Test
//    fun testCtt() {
//
//        val log = File("C:\\Users\\robotics\\Documents\\GitHub\\2020-Onseason\\log\\ctt4.csv")
//        val br = BufferedReader(FileReader(log));
//
//        var line = ""
//        var data = arrayListOf<List<String>>()
//        while (br.readLine().also { if(it != null) line = it } != null) {
//            val data_: List<String> = line.split(",")
//            data.add(data_)
//        }
//        data = ArrayList(data.subList(1, data.size))
//        val timeData = data.map { it[6] }
//
//        val poseData = arrayListOf<Pose2d>()
//        val gyroD = data.map { it[0] } // degrees, CW positive (need to invert)
//        val lEncD = data.map { it[4] }
//        val rEncD = data.map { it[5] }
//        val llX = data.map { it[8] }
//        val llY = data.map { it[9] }
// //        val llHeading
//        val odometry = DifferentialDriveOdometry(0.degrees.toRotation2d())
//        odometry.resetPosition(Pose2d(-0.3, -0.4, Rotation2d()), 0.5185128515.degrees.toRotation2d())
//
//        val odometryX = arrayListOf<Double>()
//        val odometryY = arrayListOf<Double>()
//        val odometryHeading = arrayListOf<Double>()
//
//        val visX = arrayListOf<Double>()
//        val visY = arrayListOf<Double>()
//
//        var firstRun = true
//        for(i in timeData.indices) {
//            if(timeData[i].isEmpty()) continue
//            val time = timeData[i].toDoubleOrNull()
//            val l = lEncD[i].toDoubleOrNull()
//            val r = rEncD[i].toDoubleOrNull()
//            val theta = gyroD[i].toDoubleOrNull()
//            // x is 8 ys 9 heading is 7
//            val visionX = llX[i].toDoubleOrNull()
//            val visionY = llY[i].toDoubleOrNull()
//
//
//
//            if(l == null || r == null || theta == null || time == null || visionX == null || visionY == null) continue
// //            theta *= -1.0
//
//            val heading = Math.IEEEremainder(theta, 360.0);
//
//            odometry.update(Rotation2d.fromDegrees(heading), l, r)
//
//            odometryX.add(odometry.poseMeters.translation.x)
//            odometryY.add(odometry.poseMeters.translation.y)
//            odometryHeading.add(odometry.poseMeters.rotation.radians)
//
//            visX.add(visionX)
//            visY.add(visionY)
//        }
//
//        val chartBuilder = XYChartBuilder()
//        chartBuilder.title = "The Magic of Sensor Fusion"
//        val chart = chartBuilder.build()
//
//        chart.addSeries("Pure odometry", odometryX, odometryY)
// //        chart.addSeries("Vision", visX, visY)
// //        chart.addSeries("Trajectory", trajXs, trajYs)
// //        chart.addSeries("xHat", observerXs, observerYs)
//
//        SwingWrapper(chart).displayChart()
//        Thread.sleep(1000000)
//    }
//
//    @Test
//    fun testSwerveTrajectoryGains() {
//        val model = LinearSystem(
//                `3`, `3`, `3`,
//                MatrixUtils.zeros(`3`, `3`),
//                MatrixUtils.eye(`3`),
//                MatrixUtils.eye(`3`),
//                MatrixUtils.zeros(`3`, `3`),
//                vec(`3`).fill(-3.5, -3.5, -3.0), // umin
//                vec(`3`).fill(3.5, 3.5, 3.0) // umax
//        )
//
//        val controller = LinearQuadraticRegulator(
//                `3`, `3`, model,
//                vec(`3`).fill(5.inches.inMeters(), 5.inches.inMeters(), 5.degrees.inRadians()),
//                vec(`3`).fill(1.0, 1.0, 2.0),
//                0.020
//        )
//
//        println(controller.k.storage)
//
//        val wheelPlant = LinearSystem.identifyVelocitySystem(2.9, 0.3, 12.0)
//        val wheelController = LinearQuadraticRegulator(
//                `1`, `1`, wheelPlant,
//                vec(`1`).fill(3.inches.inMeters()),
//                vec(`1`).fill(12.0),
//                1.0 / 1000.0
//        )
//
//        val kp = wheelController.k[0] // volts per meter per sec of error
//        // v = r w, or w = v / r
//        // so kp_rad = (volts / meter per sec) * r meters / radians
//        val kpRad = kp * 2.inches.inMeters()
//        val kpRot = kpRad / 2.0 / PI
//        val kpRotPerMinute = kpRot / 60.0
//        val kpOutputRotPerMinte = kpRotPerMinute / 12.0
//
//        println(kpOutputRotPerMinte)
//    }
//
// }
