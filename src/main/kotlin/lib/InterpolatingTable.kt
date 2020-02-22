package lib

import java.util.*
import org.ghrobotics.lib.mathematics.lerp
import org.ghrobotics.lib.types.Interpolatable

class InterpolatingTable<T : Interpolatable<T>>(vararg samples: Pair<Double, T>) {

    private val bufferMap = TreeMap<Double, T>(samples.toMap())

    fun get(distance: Double): T? {
        if (bufferMap.isEmpty()) return null

        bufferMap[distance]?.let { return it }

        val topBound = bufferMap.ceilingEntry(distance)
        val bottomBound = bufferMap.floorEntry(distance)

        return when {
            topBound == null && bottomBound == null -> null
            topBound == null -> bottomBound.value
            bottomBound == null -> topBound.value
            else -> bottomBound.value.interpolate(
                    topBound.value,
                    ((distance - bottomBound.key) / (topBound.key - bottomBound.key))
            )
        }
    }
}

fun Number.interpolate(endValue: Number, t: Double) = this.toDouble().lerp(endValue.toDouble(), t)
