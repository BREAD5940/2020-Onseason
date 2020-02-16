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

    @Test fun testShotTable() {

        val table = Constants.defaultShotLookupTable

        assertEquals(table.get(-3.9), ShotParameter(67.degrees, 4000.revolutionsPerMinute))
        assertEquals(table.get(0.3), ShotParameter(64.5.degrees, 3500.revolutionsPerMinute, (1).degrees))
        assertEquals(table.get(4.3), ShotParameter(65.degrees, 2600.revolutionsPerMinute, (1).degrees))
        assertEquals(table.get(5.4), ShotParameter(63.8.degrees, 2400.revolutionsPerMinute, (0.5).degrees))
        assertEquals(table.get(8.6), ShotParameter(62.5.degrees, 2400.revolutionsPerMinute, (0.5).degrees))
        assertEquals(table.get(12.2), ShotParameter(61.5.degrees, 2100.revolutionsPerMinute, 0.5.degrees))
        assertEquals(table.get(16.2), ShotParameter(60.5.degrees, 1900.revolutionsPerMinute, 0.5.degrees))

        assertEquals(ShotParameter(64.75.degrees, ((3500.0+2600.0) / 2.0).revolutionsPerMinute, 1.degrees), table.get(2.3))
    }
}

inline class InterpolatingDouble(val number: Double) : Interpolatable<InterpolatingDouble> {
    override fun interpolate(endValue: InterpolatingDouble, t: Double) =
            InterpolatingDouble(number.interpolate(endValue.number, t))
}

fun Number.interpolatable() = InterpolatingDouble(toDouble())
