package frc.robot.subsystems

import frc.robot.Constants
import frc.robot.subsystems.shooter.ShotParameter
import lib.InterpolatingTable
import lib.interpolate
import lib.revolutionsPerMinute
import org.ghrobotics.lib.mathematics.units.derived.degrees
import org.ghrobotics.lib.types.Interpolatable
import org.junit.Assert.*
import org.junit.Test

class InterpolatingTableTest {

    @Test fun testTable() {

        val table = InterpolatingTable(
                0.0 to 0.interpolatable(),
                2.0 to 4.interpolatable()
        )
        assertEquals(table.get(0.0)!!.number, 0.0, 0.01)
        assertEquals(table.get(2.0)!!.number, 4.0, 0.01)
        assertEquals(table.get(10.0)!!.number, 4.0, 0.01)
        assertEquals(table.get(1.0)!!.number, 2.0, 0.01)
    }
}

inline class InterpolatingDouble(val number: Double) : Interpolatable<InterpolatingDouble> {
    override fun interpolate(endValue: InterpolatingDouble, t: Double) =
            InterpolatingDouble(number.interpolate(endValue.number, t))
}

fun Number.interpolatable() = InterpolatingDouble(toDouble())
