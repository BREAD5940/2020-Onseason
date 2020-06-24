package frc.robot.subsystems

import edu.wpi.first.wpilibj.LinearFilter
import edu.wpi.first.wpilibj.MedianFilter
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator
import edu.wpi.first.wpilibj.estimator.KalmanFilter
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry
import edu.wpi.first.wpilibj.system.LinearSystem
import edu.wpi.first.wpilibj.system.plant.LinearSystemId
import edu.wpi.first.wpiutil.math.Matrix
import edu.wpi.first.wpiutil.math.MatrixUtils
import edu.wpi.first.wpiutil.math.numbers.N1
import edu.wpi.first.wpiutil.math.numbers.N3
import frc.team4069.keigen.*
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.mathematics.units.derived.inRadians
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.mathematics.units.inMeters
import org.ghrobotics.lib.mathematics.units.inches
import org.junit.Test
import org.knowm.xchart.SwingWrapper
import org.knowm.xchart.XYChartBuilder
import org.knowm.xchart.XYSeries
import org.knowm.xchart.style.Styler
import java.awt.Color
import java.io.BufferedReader
import java.io.File
import java.io.FileReader
import java.util.function.Function
import kotlin.math.PI

class StateSpaceMemes {

    @Test
    fun makeCharts() {
        val flywheel1 = LinearSystemId.identifyVelocitySystem(1.0, 0.5)
        val kf1 = KalmanFilter(`1`, `1`, flywheel1,
                vec(`1`).fill(3.0), vec(`1`).fill(0.01), 0.020)

        val flywheel2 = LinearSystemId.identifyVelocitySystem(1.0, 0.5)
        val kf2 = KalmanFilter(`1`, `1`, flywheel2,
                vec(`1`).fill(1.0), vec(`1`).fill(1.0), 0.020)

        var groundTruthX = vec(`1`).fill(0)
        val rand = java.util.Random(254)
        val noiseMag = 0.1

        val timeData = ArrayList<Double>()
        val xData1 = ArrayList<Double>()
        val xData2 = ArrayList<Double>()
        val yData = ArrayList<Double>()
        val groundTruthData = ArrayList<Double>()

        for (i in 0..200) {
            val y = groundTruthX + vec(`1`).fill(rand.nextGaussian() * noiseMag)
            kf1.correct(vec(`1`).fill(0), y)
            kf2.correct(vec(`1`).fill(0), y)

            timeData.add(i * 0.020)
            xData1.add(kf1.getXhat(0))
            xData2.add(kf2.getXhat(0))
            yData.add(y[0])
            groundTruthData.add(groundTruthX[0])

            kf1.predict(vec(`1`).fill(0), 0.020)
            kf2.predict(vec(`1`).fill(0), 0.020)
//            groundTruthX += vec(`1`).fill(0.020)
        }
        val chart = XYChartBuilder().build()
        chart.title = "Kalman Filter Comparison"
        chart.addSeries("Ground truth", timeData, groundTruthData)
        chart.addSeries("Y", timeData, yData)
        chart.addSeries("KF1, State std dev=3, Measurement std dev=0.01", timeData, xData1)
        chart.addSeries("KF2, State std dev=1, Measurement std dev=1", timeData, xData2)

        SwingWrapper(chart).displayChart()
        Thread.sleep(1000000)
    }

    @Test
    fun graphFlywheelData() {
        val log = File("C:\\Users\\robotics\\Downloads\\flywheel.csv")
        val br = BufferedReader(FileReader(log));

        var line = ""
        var data = arrayListOf<List<String>>()
        while (br.readLine().also { if (it != null) line = it } != null) {
            val data_: List<String> = line.split(",")
            data.add(data_)
        }
        data = ArrayList(data.subList(1, data.size))
        val timeData = data.map { it[0].toDouble() }
        val refData = data.map { it[1].toDouble() }
        val measurementData = data.map { it[2].toDouble() }
        val voltData = data.map { it[4].toDouble() }

//        val flywheel1 = LinearSystem.identifyVelocitySystem(0.002, 0.007, 12.0)
        val flywheel1 = LinearSystemId.identifyVelocitySystem(0.0022, 0.005)

        val kf1 = KalmanFilter(`1`, `1`, flywheel1,
                vec(`1`).fill(3.0), vec(`1`).fill(0.2), 0.020)

        val flywheel2 = LinearSystemId.identifyVelocitySystem(0.0022, 0.005)

        val kf2 = KalmanFilter(`1`, `1`, flywheel2,
                vec(`1`).fill(2.0), vec(`1`).fill(1.0), 0.020)

        val linearFilter = LinearFilter.singlePoleIIR(0.1, 0.020)
        val movingMedianFilter = MedianFilter(10)

        val xData1 = ArrayList<Double>()
        val xData2 = ArrayList<Double>()

        val linFilterData = ArrayList<Double>()
        val medianFilterData = ArrayList<Double>()
        kf1.setXhat(vec(`1`).fill(measurementData[0]))
        var cVolts = 0.0
        timeData.forEachIndexed { i, _ ->
            kf1.correct(vec(`1`).fill(cVolts), vec(`1`).fill(measurementData[i]))
            kf2.correct(vec(`1`).fill(cVolts), vec(`1`).fill(measurementData[i]))

            xData1.add(kf1.getXhat(0))
            xData2.add(kf2.getXhat(0))

            cVolts = voltData[i]
            kf1.predict(vec(`1`).fill(cVolts), 0.020)
            kf2.predict(vec(`1`).fill(cVolts), 0.020)

            medianFilterData.add(movingMedianFilter.calculate(measurementData[i]))
            linFilterData.add(linearFilter.calculate(measurementData[i]))
        }

        val chart = XYChartBuilder().build()
        chart.title = "Kalman Filter Comparison"
        chart.styler.markerSize = 0

        // Customize Chart
        chart.styler.legendPosition = Styler.LegendPosition.InsideSE
        chart.styler.setAxisTitlesVisible(false)
        chart.styler.defaultSeriesRenderStyle = XYSeries.XYSeriesRenderStyle.Line

        // Customize Chart
        chart.styler.plotBackgroundColor = Color(40, 42, 54)
        chart.styler.plotGridLinesColor = Color(68, 71, 90)
        chart.styler.chartBackgroundColor = Color(46, 49, 62)
        chart.styler.plotBorderColor = Color(68, 71, 90)
        chart.styler.legendBackgroundColor = Color(68, 71, 90)
        chart.styler.axisTickLabelsColor = Color(248, 248, 242)
        chart.styler.axisTickMarksColor = Color(68, 71, 90)
        chart.styler.chartFontColor = Color(248, 248, 242)
        chart.styler.isChartTitleBoxVisible = false
        chart.styler.isPlotGridLinesVisible = true

        chart.styler.isLegendVisible = true

        chart.addSeries("refData", timeData, refData).lineColor = Color(139, 233, 253)
        chart.addSeries("measurement", timeData, measurementData).lineColor = Color(180, 180, 180)
        chart.addSeries("xhat, state = 3.0, measurement = 0.2", timeData, xData1).lineColor = Color.orange
        chart.addSeries("xhat, state = 2.0, measurement = 1.0", timeData, xData2).lineColor = Color.MAGENTA
        chart.addSeries("Median filter, window of 10", timeData, medianFilterData).lineColor = Color.red
        chart.addSeries("IIR filter, time const 0.1", timeData, linFilterData).lineColor = Color(255, 121, 198)

//        val chart2 = XYChartBuilder().build()
//        chart2.addSeries("voltage", timeData, voltData)
//        SwingWrapper(listOf(chart, chart2)).displayChartMatrix()
        SwingWrapper(chart).displayChart()
        Thread.sleep(1000000)
    }

    @Test
    fun graphP() {
        val log = File("C:\\Users\\robotics\\Downloads\\flywheel.csv")
        val br = BufferedReader(FileReader(log));

        var line = ""
        var data = arrayListOf<List<String>>()
        while (br.readLine().also { if (it != null) line = it } != null) {
            val data_: List<String> = line.split(",")
            data.add(data_)
        }
        data = ArrayList(data.subList(1, data.size))
        val timeData = data.map { it[0].toDouble() }
        val refData = data.map { it[1].toDouble() }
        val measurementData = data.map { it[2].toDouble() }
        val voltData = data.map { it[4].toDouble() }

        val flywheel1 = LinearSystemId.identifyVelocitySystem(0.0022, 0.005)
        val flywheel2 = LinearSystemId.identifyVelocitySystem(0.0022, 0.005)

        val kf1 = KalmanFilter(`1`, `1`, flywheel1,
                vec(`1`).fill(1.0), vec(`1`).fill(0.1), 0.020)
        kf1.xhat = vec(`1`).fill(measurementData[0])

        val kf2 = KalmanFilter(`1`, `1`, flywheel2,
                vec(`1`).fill(0.1), vec(`1`).fill(0.01), 0.020)
        kf2.xhat = vec(`1`).fill(measurementData[0])

        val xData1 = ArrayList<Double>()
        val xData2 = ArrayList<Double>()
        val pData = ArrayList<Double>()
        val pData2 = ArrayList<Double>()

        var cVolts = 0.0
        timeData.forEachIndexed { i, _ ->
            if (i % 12 == 0) kf1.correct(vec(`1`).fill(cVolts), vec(`1`).fill(measurementData[i]))
            if (i % 12 == 0) kf2.correct(vec(`1`).fill(cVolts), vec(`1`).fill(measurementData[i]))

            xData1.add(kf1.getXhat(0))
            xData2.add(kf2.getXhat(0))
            pData.add(kf1.p[0])
            pData2.add(kf2.p[0])

            cVolts = voltData[i]
            kf1.predict(vec(`1`).fill(cVolts), 0.020)
            kf2.predict(vec(`1`).fill(cVolts), 0.020)
        }

        val chart = XYChartBuilder().build()
        chart.title = "KF with correct() called every 12th iteration"
        chart.styler.markerSize = 0

        // Customize Chart
        chart.styler.legendPosition = Styler.LegendPosition.InsideSE
        chart.styler.setAxisTitlesVisible(false)
        chart.styler.defaultSeriesRenderStyle = XYSeries.XYSeriesRenderStyle.Line

        // Customize Chart
        chart.styler.plotBackgroundColor = Color(40, 42, 54)
        chart.styler.plotGridLinesColor = Color(68, 71, 90)
        chart.styler.chartBackgroundColor = Color(46, 49, 62)
        chart.styler.plotBorderColor = Color(68, 71, 90)
        chart.styler.legendBackgroundColor = Color(68, 71, 90)
        chart.styler.axisTickLabelsColor = Color(248, 248, 242)
        chart.styler.axisTickMarksColor = Color(68, 71, 90)
        chart.styler.chartFontColor = Color(248, 248, 242)
        chart.styler.isChartTitleBoxVisible = false
        chart.styler.isPlotGridLinesVisible = true
        chart.styler.isLegendVisible = true

        val chart2 = XYChartBuilder().build()
        chart2.styler.markerSize = 0
        chart2.styler.legendPosition = Styler.LegendPosition.InsideSE
        chart2.styler.setAxisTitlesVisible(false)
        chart2.styler.defaultSeriesRenderStyle = XYSeries.XYSeriesRenderStyle.Line
        chart2.styler.plotBackgroundColor = Color(40, 42, 54)
        chart2.styler.plotGridLinesColor = Color(68, 71, 90)
        chart2.styler.chartBackgroundColor = Color(46, 49, 62)
        chart2.styler.plotBorderColor = Color(68, 71, 90)
        chart2.styler.legendBackgroundColor = Color(68, 71, 90)
        chart2.styler.axisTickLabelsColor = Color(248, 248, 242)
        chart2.styler.axisTickMarksColor = Color(68, 71, 90)
        chart2.styler.chartFontColor = Color(248, 248, 242)
        chart2.styler.isChartTitleBoxVisible = false
        chart2.styler.isPlotGridLinesVisible = true
        chart2.styler.isLegendVisible = true

        val chart3 = XYChartBuilder().build()
        chart3.styler.markerSize = 0
        chart3.styler.legendPosition = Styler.LegendPosition.InsideSE
        chart3.styler.setAxisTitlesVisible(false)
        chart3.styler.defaultSeriesRenderStyle = XYSeries.XYSeriesRenderStyle.Line
        chart3.styler.plotBackgroundColor = Color(40, 42, 54)
        chart3.styler.plotGridLinesColor = Color(68, 71, 90)
        chart3.styler.chartBackgroundColor = Color(46, 49, 62)
        chart3.styler.plotBorderColor = Color(68, 71, 90)
        chart3.styler.legendBackgroundColor = Color(68, 71, 90)
        chart3.styler.axisTickLabelsColor = Color(248, 248, 242)
        chart3.styler.axisTickMarksColor = Color(68, 71, 90)
        chart3.styler.chartFontColor = Color(248, 248, 242)
        chart3.styler.isChartTitleBoxVisible = false
        chart3.styler.isPlotGridLinesVisible = true
        chart3.styler.isLegendVisible = true

        chart2.styler.yAxisMin = 0.0
        chart2.styler.yAxisMax = 0.4
        chart3.styler.yAxisMin = 0.0
        chart3.styler.yAxisMax = 0.4

        chart.addSeries("Reference, RPM", timeData, refData).lineColor = Color(139, 233, 253)
        chart.addSeries("Measurement, RPM", timeData, measurementData).lineColor = Color(180, 180, 180)
        chart.addSeries("Estaimated state, RPM", timeData, xData1).lineColor = Color.orange
        chart2.addSeries("Error covariance, rad/sec squared with q=[1.0], r=[0.1]", timeData, pData).lineColor = Color.MAGENTA
        chart3.addSeries("Error covariance 2, rad/sec squared with q=[0.1], r=[0.01]", timeData, pData2).lineColor = Color.MAGENTA
        SwingWrapper(listOf(chart, chart2, chart3)).displayChartMatrix("KF with correct() called every 12th iteration")
        Thread.sleep(1000000)
    }

    @Test
    fun testCtt() {

        val log = File("C:\\Users\\robotics\\Documents\\GitHub\\2020-Onseason\\log\\ctt4.csv")
        val br = BufferedReader(FileReader(log));

        var line = ""
        var data = arrayListOf<List<String>>()
        while (br.readLine().also { if (it != null) line = it } != null) {
            val data_: List<String> = line.split(",")
            data.add(data_)
        }
        data = ArrayList(data.subList(1, data.size))
        val timeData = data.map { it[6] }

        val poseData = arrayListOf<Pose2d>()
        val gyroD = data.map { it[0] } // degrees, CW positive (need to invert)
        val lEncD = data.map { it[4] }
        val rEncD = data.map { it[5] }
        val llX = data.map { it[8] }
        val llY = data.map { it[9] }
//        val llHeading
        val odometry = DifferentialDriveOdometry(0.degrees.toRotation2d())
        odometry.resetPosition(Pose2d(-0.3, -0.4, Rotation2d()), 0.5185128515.degrees.toRotation2d())

        val odometryX = arrayListOf<Double>()
        val odometryY = arrayListOf<Double>()
        val odometryHeading = arrayListOf<Double>()

        val visX = arrayListOf<Double>()
        val visY = arrayListOf<Double>()

        var firstRun = true
        for (i in timeData.indices) {
            if (timeData[i].isEmpty()) continue
            val time = timeData[i].toDoubleOrNull()
            val l = lEncD[i].toDoubleOrNull()
            val r = rEncD[i].toDoubleOrNull()
            val theta = gyroD[i].toDoubleOrNull()
            // x is 8 ys 9 heading is 7
            val visionX = llX[i].toDoubleOrNull()
            val visionY = llY[i].toDoubleOrNull()



            if (l == null || r == null || theta == null || time == null || visionX == null || visionY == null) continue
//            theta *= -1.0

            val heading = Math.IEEEremainder(theta, 360.0);

            odometry.update(Rotation2d.fromDegrees(heading), l, r)

            odometryX.add(odometry.poseMeters.translation.x)
            odometryY.add(odometry.poseMeters.translation.y)
            odometryHeading.add(odometry.poseMeters.rotation.radians)

            visX.add(visionX)
            visY.add(visionY)
        }

        val chartBuilder = XYChartBuilder()
        chartBuilder.title = "The Magic of Sensor Fusion"
        val chart = chartBuilder.build()

        chart.addSeries("Pure odometry", odometryX, odometryY)
//        chart.addSeries("Vision", visX, visY)
//        chart.addSeries("Trajectory", trajXs, trajYs)
//        chart.addSeries("xHat", observerXs, observerYs)

        SwingWrapper(chart).displayChart()
        Thread.sleep(1000000)
    }

    @Test
    fun testSwerveTrajectoryGains() {
        val model = LinearSystem(
                `3`, `3`, `3`,
                MatrixUtils.zeros(`3`, `3`),
                MatrixUtils.eye(`3`),
                MatrixUtils.eye(`3`),
                MatrixUtils.zeros(`3`, `3`))

        val controller = LinearQuadraticRegulator(
                model,
                vec(`3`).fill(5.inches.inMeters(), 5.inches.inMeters(), 5.degrees.inRadians()),
                vec(`3`).fill(1.0, 1.0, 2.0),
                0.020
        )

        println(controller.k.storage)

        val wheelPlant = LinearSystemId.identifyVelocitySystem(2.9, 0.3)
        val wheelController = LinearQuadraticRegulator(
                wheelPlant,
                vec(`1`).fill(3.inches.inMeters()),
                vec(`1`).fill(12.0),
                1.0 / 1000.0
        )

        val kp = wheelController.k[0] // volts per meter per sec of error
        // v = r w, or w = v / r
        // so kp_rad = (volts / meter per sec) * r meters / radians
        val kpRad = kp * 2.inches.inMeters()
        val kpRot = kpRad / 2.0 / PI
        val kpRotPerMinute = kpRot / 60.0
        val kpOutputRotPerMinte = kpRotPerMinute / 12.0

        println(kpOutputRotPerMinte)
    }

}