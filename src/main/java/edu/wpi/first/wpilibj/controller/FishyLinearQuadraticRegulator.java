package edu.wpi.first.wpilibj.controller;

import edu.wpi.first.wpilibj.math.Drake;
import edu.wpi.first.wpilibj.math.StateSpaceUtils;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpiutil.math.*;
import edu.wpi.first.wpiutil.math.numbers.N1;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.simple.SimpleMatrix;

public class FishyLinearQuadraticRegulator<States extends Num, Inputs extends Num,
        Outputs extends Num> {

    private final Matrix<States, States> m_A;
    private final Matrix<States, Inputs> m_B;

    private boolean m_enabled = false;

    /**
     * The current reference state
     */
    private Matrix<States, N1> m_r;

    /**
     * The computed and capped controller output
     */
    private Matrix<Inputs, N1> m_u;

    // Controller gain.
    public Matrix<Inputs, States> m_K;

    // disc b
    private Matrix<States, Inputs> m_discB;
    private Matrix<States, States> m_discA;

    public Matrix<Inputs, N1> getU() {
        return m_u;
    }

    public Matrix<States, N1> getR() {
        return m_r;
    }

    /**
     * Constructs a controller with the given coefficients and plant.
     *
     * @param plant The plant being controlled.
     * @param qElms The maximum desired error tolerance for each state.
     * @param rElms The maximum desired control effort for each input.
     * @param dtSeconds     Discretization timestep.
     */
    public FishyLinearQuadraticRegulator(
            Nat<States> states, Nat<Inputs> inputs,
            LinearSystem<States, Inputs, Outputs> plant,
            Matrix<States, N1> qElms,
            Matrix<Inputs, N1> rElms,
            double dtSeconds
    ) {
        this(states, inputs, plant.getA(), plant.getB(), qElms, rElms, dtSeconds);
    }

    /**
     * Constructs a controller with the given coefficients and plant.
     *
     * @param a      Continuous system matrix of the plant being controlled.
     * @param b      Continuous input matrix of the plant being controlled.
     * @param qElems The maximum desired error tolerance for each state.
     * @param rElems The maximum desired control effort for each input.
     * @param dtSeconds     Discretization timestep.
     */
    public FishyLinearQuadraticRegulator(
            Nat<States> states, Nat<Inputs> inputs,
            Matrix<States, States> a, Matrix<States, Inputs> b,
            Matrix<States, N1> qElems, Matrix<Inputs, N1> rElems,
            double dtSeconds
    ) {
        this.m_A = a;
        this.m_B = b;

        var size = states.getNum() + inputs.getNum();
        var Mcont = new SimpleMatrix(0, 0);
        var scaledA = m_A.times(dtSeconds);
        var scaledB = m_B.times(dtSeconds);
        Mcont = Mcont.concatColumns(scaledA.getStorage());
        Mcont = Mcont.concatColumns(scaledB.getStorage());
        // so our Mcont is now states x (states + inputs)
        // and we want (states + inputs) x (states + inputs)
        // so we want to add (inputs) many rows onto the bottom
        Mcont = Mcont.concatRows(new SimpleMatrix(inputs.getNum(), size));

        // calculate discrete A and B matrices
        SimpleMatrix Mstate = StateSpaceUtils.exp(Mcont);

        var discA = new SimpleMatrix(states.getNum(), states.getNum());
        var discB = new SimpleMatrix(states.getNum(), inputs.getNum());
        CommonOps_DDRM.extract(Mstate.getDDRM(), 0, 0, discA.getDDRM());
        CommonOps_DDRM.extract(Mstate.getDDRM(), 0, states.getNum(), discB.getDDRM());

        // make the cost matrices
        var Q = StateSpaceUtils.makeCostMatrix(states, qElems);
        var R = StateSpaceUtils.makeCostMatrix(inputs, rElems);

        this.m_discB = new Matrix<>(discB);
        this.m_discA = new Matrix<>(discA);

        var S = Drake.discreteAlgebraicRiccatiEquation(discA, discB, Q.getStorage(), R.getStorage());

//        var temp = (m_discB.transpose().getStorage().mult(S).mult(discB)).plus(R.getStorage());
//        m_K = new Matrix<>(
//            StateSpaceUtils.lltDecompose(temp).solve(discB.transpose().mult(S).mult(discA)));

        m_K = new Matrix<>((discB.transpose().mult(S).mult(discB).plus(R.getStorage())).invert().mult(discB.transpose()).mult(S).mult(discA)); // TODO (HIGH) SWITCH ALGORITHMS

        this.m_r = new Matrix<>(new SimpleMatrix(states.getNum(), 1));
        this.m_u = new Matrix<>(new SimpleMatrix(inputs.getNum(), 1));

    }

    /**
     * Enables the controller
     */
    public void enable() {
        m_enabled = true;
    }

    /**
     * Disables controller and zeros controller output U.
     */
    public void disable() {
        m_enabled = false;
        m_u.getStorage().fill(0.0);
    }

    public boolean isEnabled() {
        return m_enabled;
    }

    /**
     * Returns the controller matrix K.
     */
    public Matrix<Inputs, States> getK() {
        return m_K;
    }

    /**
     * Resets the controller.
     */
    public void reset() {
        m_r.getStorage().fill(0.0);
        m_u.getStorage().fill(0.0);
    }

    /**
     * Update controller without setting a new reference.
     *
     * @param x The current state x.
     */
    public void update(Matrix<States, N1> x) {
        if(m_enabled) {
            m_u = m_K.times(m_r.minus(x)).plus(new Matrix<>(StateSpaceUtils.householderQrDecompose(m_discB.getStorage()).solve((m_r.minus(m_discA.times(m_r))).getStorage())));
        }
    }

    /**
     * Set a new reference and update the controller.
     *
     * @param x The current state x.
     * @param nextR the next reference vector r.
     */
    public void update(Matrix<States, N1> x, Matrix<States, N1> nextR) {
        if(m_enabled) {
            Matrix<States, N1> error = m_r.minus(x);
            Matrix<Inputs, N1> feedBack = m_K.times(error);
            Matrix<Inputs, N1> feedForward = new Matrix<>(StateSpaceUtils.householderQrDecompose(m_discB.getStorage()).solve((nextR.minus(m_discA.times(m_r))).getStorage()));

            m_u = feedBack.plus(feedForward);
            m_r = nextR;
        }
    }

}
