package frc.robot.subsystems
import frc.robot.auto.paths.TrajectoryFactory
import org.junit.Test
import org.knowm.xchart.SwingWrapper
import org.knowm.xchart.XYChart
import org.knowm.xchart.XYChartBuilder

class TrajectoryDisplayTest {

    @Test
    fun main() {
        var x: ArrayList<Double> = ArrayList()
        var y: ArrayList<Double> = ArrayList()
        TrajectoryFactory.grabThreeAndShoot.states.forEach {
            x.add(it.poseMeters.translation.x)
            y.add(it.poseMeters.translation.y)
        }
        // Now X and Y arrays are built! Yay!

        val chart = XYChart(XYChartBuilder())
        chart.addSeries("Trajectory", x, y)
        (SwingWrapper(chart)).displayChart()
        return
    }
}
