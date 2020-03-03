package frc.robot.subsystems.drive.localization

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.estimator.KalmanFilter
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.system.LinearSystem
import edu.wpi.first.wpiutil.math.numbers.N3
import frc.team4069.keigen.*
import frc.team4069.keigen.Vector
import java.util.*
import lib.interpolate
import org.ghrobotics.lib.localization.TimePoseInterpolatableBuffer
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derived.radians
import org.ghrobotics.lib.mathematics.units.derived.toRotation2d
import org.ghrobotics.lib.mathematics.units.operations.div
import org.ghrobotics.lib.utils.Source

/**
 * A kalman filter designed to run at 200hz
 */
object DrivetrainLocalizer {

    private const val dt = 0.005

    private val observer = KalmanFilter(`3`, `3`, `3`,
            LinearSystem(`3`, `3`, `3`,
                    zeros(`3`, `3`),
                    eye(`3`),
                    eye(`3`),
                    zeros(`3`, `3`),
                    vec(`3`).fill(-4.0, -4.0, -12.0),
                    vec(`3`).fill(4.0, 4.0, 12.0)),
            vec(`3`).fill(0.1, 0.1, 0.1), // state weights
            vec(`3`).fill(4.0, 4.0, 4.0), // measurement weights
            dt)

    private val pastPoseBuffer = TimePoseInterpolatableBuffer()
    private val pastInputs = TreeMap<SIUnit<Second>, Vector<N3>>()

    val estimatedPosition get() = observer.xhat.toPose()

    fun reset() {
        pastPoseBuffer.clear()
        pastInputs.clear()
        observer.reset()
    }

    fun update(velocity: ChassisSpeeds, measuredPose: Pose2d? = null, timestamp: SIUnit<Second> = (-1).seconds) =
            update(vec(`3`).fill(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond, velocity.omegaRadiansPerSecond),
                    measuredPose, timestamp)

    /**
     * Update the filter. Call this periodically at 200hz!
     */
    fun update(nowInput: Vector<N3>, measuredPose: Pose2d? = null, timestamp: SIUnit<Second> = (-1).seconds) {

        // if we see a target, roll back the filter, apply the measurement and roll forward
        if (measuredPose != null) {
            // Latency compensation; reset observer pose to the pose when the frame was in sight,
            // add correction factor, and replay all predictions into the future
            val thenPose = pastPoseBuffer[timestamp]
            if (thenPose != null) {

                val firstIdx = pastInputs.keys.indexOf(timestamp)

                if (pastInputs.keys.size >= 2 && firstIdx != -1) {
                    val inputsToReplay = pastInputs.values.toList().subList(firstIdx, pastInputs.size - 1)

                    observer.xhat = thenPose.toVec()

                    observer.correct(pastInputs.values.toList()[firstIdx], measuredPose.toVec())

                    for (v in inputsToReplay) {
                        observer.predict(v, dt)
                    }
                }
            }
        }
        observer.predict(nowInput, dt)

        val now = Timer.getFPGATimestamp().second
        val iterator = pastInputs.iterator()
        iterator.forEach {
            if (now - it.key >= 1.second) {
                iterator.remove()
            }
        }

        pastInputs[now] = nowInput
        pastPoseBuffer[timestamp] = observer.xhat.toPose()
    }
}

private fun Vector<N3>.toPose() = Pose2d(get(0), get(1), get(0).radians.toRotation2d())

fun Pose2d.toVec() = vec(`3`).fill(translation.x, translation.y, rotation.radians)

class TimeInterpolatableInputBuffer(private val historySpan: SIUnit<Second> = 1.0.seconds, private val timeSource: Source<SIUnit<Second>> = { Timer.getFPGATimestamp().seconds }) {

    val bufferMap = TreeMap<SIUnit<Second>, Vector<N3>>()

    operator fun set(time: SIUnit<Second>, value: Vector<N3>): Vector<N3>? {
        cleanUp()
        return bufferMap.put(time, value)
    }

    private fun cleanUp() {
        val currentTime = timeSource()

        while (bufferMap.isNotEmpty()) {
            val entry = bufferMap.lastEntry()
            if (currentTime - entry.key >= historySpan) {
                bufferMap.remove(entry.key)
            } else {
                return
            }
        }
    }

    fun clear() {
        bufferMap.clear()
    }

    operator fun get(time: SIUnit<Second>): Vector<N3>? {
        if (bufferMap.isEmpty()) return null

        bufferMap[time]?.let { return it }

        val topBound = bufferMap.ceilingEntry(time)
        val bottomBound = bufferMap.floorEntry(time)

        return when {
            topBound == null -> bottomBound.value
            bottomBound == null -> topBound.value
            else -> bottomBound.value.interpolate(
                    topBound.value,
                    ((time - bottomBound.key) / (topBound.key - bottomBound.key)).unitlessValue
            )
        }
    }
}

private fun Vector<N3>.interpolate(upperBound: Vector<N3>, t: Double) = vec(`3`).fill(
        get(0).interpolate(upperBound[0], t),
        get(0).interpolate(upperBound[0], t),
        get(0).interpolate(upperBound[0], t)
)
