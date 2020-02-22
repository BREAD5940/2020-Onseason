package frc.robot.subsystems

import frc.robot.auto.paths.TrajectoryFactory
import frc.robot.autonomous.routines.TenPCAutoRoutine
import java.io.File
import javax.imageio.ImageIO
import javax.swing.JFrame
import org.ghrobotics.lib.mathematics.units.kFeetToMeter
import org.jfree.chart.ChartFactory
import org.jfree.chart.ChartPanel
import org.jfree.chart.plot.PlotOrientation
import org.jfree.data.xy.XYDataItem
import org.jfree.data.xy.XYSeriesCollection
import org.junit.Test

class TrajectoryDisplayTest {

    @Test fun testDuration() {
        println(TenPCAutoRoutine().duration)
    }

    @Test
    fun displayTrajectory() {
        var seriesX = org.jfree.data.xy.XYSeries("x")
        TrajectoryFactory.eightPCAutoShootToShieldGenerator.states.forEach {
            seriesX.add(XYDataItem(it.poseMeters.translation.x / kFeetToMeter, it.poseMeters.translation.y / kFeetToMeter))
        }

        // Now X and Y arrays are built! Yay!
        val collection = XYSeriesCollection()
        collection.addSeries(seriesX)
        val chart = ChartFactory.createScatterPlot(
                "Robot Chart.", "X", "Y", collection, PlotOrientation.VERTICAL, false, false, false
        )
        chart.xyPlot.domainAxis.isAutoRange = false
        chart.xyPlot.rangeAxis.isAutoRange = false
        chart.xyPlot.isDomainGridlinesVisible = false
        chart.xyPlot.isRangeGridlinesVisible = false
        chart.xyPlot.rangeAxis.setRange(0.0, 27.0)
        chart.xyPlot.domainAxis.setRange(0.0, 54.0)
        chart.plot.backgroundImage = ImageIO.read(File("src/test/resources/chart-background.png"))
        var panel = ChartPanel(chart)
        val frame = JFrame()
        frame.add(panel)
        frame.setSize(2598 / 3, 1299 / 3)
        frame.setVisible(true)
//        Thread.sleep(100000)
        return
    }
}
