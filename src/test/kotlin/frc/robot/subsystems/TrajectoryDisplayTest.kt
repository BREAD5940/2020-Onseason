package frc.robot.subsystems

import frc.robot.auto.paths.TrajectoryFactory
import org.jfree.chart.ChartFactory
import org.jfree.chart.ChartPanel
import org.jfree.chart.JFreeChart
import org.jfree.chart.plot.PlotOrientation
import org.jfree.chart.plot.XYPlot
import org.jfree.chart.renderer.xy.XYLineAndShapeRenderer
import org.jfree.data.category.CategoryDataset
import org.jfree.data.category.DefaultCategoryDataset
import org.jfree.data.xy.XYDataItem
import org.jfree.data.xy.XYSeries
import org.jfree.data.xy.XYSeriesCollection
import org.junit.Test
import java.io.File
import javax.imageio.ImageIO
import javax.swing.JFrame


class TrajectoryDisplayTest {
    @Test
    fun main() {
        var seriesX = XYSeries("x")

        TrajectoryFactory.grabThreeAndShoot.states.forEach {
            seriesX.add(XYDataItem(  it.poseMeters.translation.x, it.poseMeters.translation.y))
        }
        // Now X and Y arrays are built! Yay!
        val collection = XYSeriesCollection()
        collection.addSeries(seriesX)
        val chart  = ChartFactory.createXYLineChart(
                "Robot Chart.", "X", "Y", collection, PlotOrientation.HORIZONTAL, false, false, false
        )
        chart.plot.backgroundImage = ImageIO.read(File("src/test/resources/chart-background.png"))
        chart.xyPlot.domainAxis.upperBound =
        var panel = ChartPanel(chart)
        val frame = JFrame()
        frame.add(panel)
        frame.setSize(2598/3,1299/3)
        frame.setVisible(true)
        Thread.sleep(100000)
        //return
    }
}
