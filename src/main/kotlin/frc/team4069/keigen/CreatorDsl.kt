package frc.team4069.keigen

import edu.wpi.first.wpiutil.math.*
import edu.wpi.first.wpiutil.math.numbers.N1
import org.ejml.dense.row.CommonOps_DDRM
import org.ejml.dense.row.factory.DecompositionFactory_DDRM
import org.ejml.simple.SimpleMatrix

fun <R : Num, C : Num> zeros(rows: Nat<R>, cols: Nat<C>): Matrix<R, C> = Matrix(SimpleMatrix(rows.num, cols.num))
fun <D : Num> zeros(size: Nat<D>): Matrix<D, N1> = Matrix(SimpleMatrix(size.num, 1))

fun <D : Num> eye(size: Nat<D>): Matrix<D, D> = Matrix(SimpleMatrix.identity(size.num))

fun <R : Num, C : Num> ones(rows: Nat<R>, cols: Nat<C>): Matrix<R, C> {
    val out = SimpleMatrix(rows.num, cols.num)
    CommonOps_DDRM.fill(out.ddrm, 1.0)
    return Matrix(out)
}
fun <D : Num> ones(size: Nat<D>): Matrix<D, N1> {
    val out = SimpleMatrix(size.num, 1)
    CommonOps_DDRM.fill(out.ddrm, 1.0)
    return Matrix(out)
}

fun <R : Num, C : Num> mat(rows: Nat<R>, cols: Nat<C>) = MatBuilder(rows, cols)

// typealias Vector<D> = Matrix<D, N1>

operator fun <D : Num> Matrix<D, N1>.get(i: Int) = storage[i, 0]
operator fun <D : Num> Matrix<D, N1>.set(i: Int, j: Double) {
    storage[i, 0] = j
}

fun <D : Num> vec(dim: Nat<D>) = VecBuilder(dim)

class VecBuilder<D : Num>(val dim: Nat<D>) {
    fun fill(vararg data: Double): Vector<D> {
        if (data.size != dim.num) {
            throw IllegalArgumentException("Invalid number of elements for ${dim.num}-dimensional vector. got ${data.size} elements")
        }

        return Vector(SimpleMatrix(dim.num, 1, false, data))
    }

    fun fill(vararg data: Int): Vector<D> {
        if (data.size != dim.num) {
            throw IllegalArgumentException("Invalid number of elements for ${dim.num}-dimensional vector. got ${data.size} elements")
        }

        return Vector(SimpleMatrix(dim.num, 1, false, data.map { it.toDouble() }.toDoubleArray()))
    }
}

fun SimpleMatrix.chol() = DecompositionFactory_DDRM.chol(this.numCols(), true).also {
    it.decompose(this.ddrm.copy())
}

fun SimpleMatrix.LU() = DecompositionFactory_DDRM.lu(this.numRows(), this.numCols()).also {
    it.decompose(this.ddrm.copy())
}

fun SimpleMatrix.QR() = DecompositionFactory_DDRM.qr(this.numRows(), this.numCols()).also {
    it.decompose(this.ddrm.copy())
}
