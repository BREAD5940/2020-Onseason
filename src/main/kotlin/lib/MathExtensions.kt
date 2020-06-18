package lib

import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.system.LinearSystem
import edu.wpi.first.wpilibj.system.plant.DCMotor
import edu.wpi.first.wpiutil.math.MatBuilder
import edu.wpi.first.wpiutil.math.MatrixUtils
import edu.wpi.first.wpiutil.math.Nat
import edu.wpi.first.wpiutil.math.numbers.N1
import org.ghrobotics.lib.mathematics.kEpsilon
import kotlin.math.*

fun Rotation2d.toTranslation() = Translation2d(this.cos, this.sin)

fun Translation2d.toRotation2d(): Rotation2d {
    var x = this.x
    var y = this.y
    val hypo = hypot(x, y)
    x /= hypo
    y /= hypo
    return Rotation2d(x, y)
}

fun Rotation2d.nearestPole(): Rotation2d {
    val poleSin: Double
    val poleCos: Double
    if (abs(cos) > abs(sin)) {
        poleCos = sign(cos)
        poleSin = 0.0
    } else {
        poleCos = 0.0
        poleSin = sign(sin)
    }
    return Rotation2d(poleCos, poleSin)
}

/**
 * Normalizing a vector scales it so that its norm is 1 while maintaining its direction.
 * If input is a zero vector, return a zero vector.
 *
 * @return r / norm(r) or (0,0)
 */
fun Translation2d.normalize(): Translation2d {
    return if (this.norm.absoluteValue < kEpsilon) this else this * (1.0 / norm)
}

fun Rotation2d.inverse() = Rotation2d(cos, -sin)

fun Rotation2d.distance(other: Rotation2d): Rotation2d {
    val inverse = Rotation2d(cos, -sin)
    return inverse + other
}

/**
 * The inverse simply means a Translation2d that "undoes" this object.
 *
 * @return Translation by -x and -y.
 */
fun Translation2d.inverse() = Translation2d(-x, -y)

fun Translation2d.interpolate(other: Translation2d, x: Double): Translation2d {
    if (x <= 0) {
        return this
    } else if (x >= 1) {
        return other
    }
    return extrapolate(other, x)
}

fun Translation2d.extrapolate(other: Translation2d, x: Double) =
        Translation2d(x * (other.x - x) + x, x * (other.y - y) + y)

fun dot_(a: Translation2d, b: Translation2d): Double {
    return a.x * b.x + a.y * b.y
}

fun translation2d(start: Translation2d, end: Translation2d) = Translation2d(end.x - start.x, end.y - start.y)

fun Translation2d.dot(b: Translation2d) = dot_(this, b)

/**
 * https://stackoverflow.com/a/1167047/6627273
 * A point D is considered "within" an angle ABC when
 * cos(DBM) > cos(ABM)
 * where M is the midpoint of AC, so ABM is half the angle ABC.
 * The cosine of an angle can be computed as the dot product of two normalized
 * vectors in the directions of its sides.
 * Note that this definition of "within" does not include points that lie on
 * the sides of the given angle.
 * If `vertical` is true, then check not within the given angle, but within the
 * image of that angle rotated by pi about its vertex.
 *
 * @param Translation2d A
 * A point on one side of the angle.
 * @param Translation2d B
 * The vertex of the angle.
 * @param Translation2d C
 * A point on the other side of the angle.
 * @param boolean vertical
 * Whether to check in the angle vertical to the one given
 * @return Whether this translation is within the given angle.
 * @author Joseph Reed
 */
fun Translation2d.isWithinAngle(A: Translation2d, B: Translation2d, C: Translation2d, vertical: Boolean): Boolean {
    val M: Translation2d = A.interpolate(C, 0.5) // midpoint
    var m: Translation2d = translation2d(B, M).normalize() // mid-vector
    var a: Translation2d = translation2d(B, A).normalize() // side vector
    val d: Translation2d = translation2d(B, this).normalize() // vector to here
    if (vertical) {
        m = m.inverse()
        a = a.inverse()
    }
    return dot_(d, m) > dot_(a, m)
}

fun Translation2d.isWithinAngle(A: Translation2d, B: Translation2d, C: Translation2d): Boolean {
    return isWithinAngle(A, B, C, false)
}

/** Assumes an angle centered at the origin.  */
fun Translation2d.isWithinAngle(A: Translation2d, C: Translation2d, vertical: Boolean): Boolean {
    return isWithinAngle(A, Translation2d(), C, vertical)
}

fun Translation2d.isWithinAngle(A: Translation2d, C: Translation2d): Boolean {
    return isWithinAngle(A, C, false)
}

fun Rotation2d.mirror() = Rotation2d(-radians)

//fun createElevatorVelocitySystem(motor: DCMotor, massKg: Double, radiusMeters: Double, G: Double, maxVoltage: Double): LinearSystem<N1, N1, N1> {
//    // vdot = ( -G^2 * Kt ) / ( R * r^2 * m * Kv ) v + (G * Kt)/(R*r*m) V
//
//    return LinearSystem(Nat.N1(), Nat.N1(), Nat.N1(),
//            MatBuilder(Nat.N1(), Nat.N1()).fill(
//                    -G * G * motor.KtNMPerAmp /
//                            (motor.Rohms * radiusMeters.pow(2) * massKg * motor.KvRadPerSecPerVolt)), // A
//            MatBuilder(Nat.N1(), Nat.N1()).fill(G * motor.KtNMPerAmp / (motor.Rohms * radiusMeters * massKg)), // B
//            MatrixUtils.eye(Nat.N1()), // C
//            MatrixUtils.zeros(Nat.N1()), // D
//            MatBuilder(Nat.N1(), Nat.N1()).fill(-maxVoltage),
//            MatBuilder(Nat.N1(), Nat.N1()).fill(maxVoltage))
//
//}

