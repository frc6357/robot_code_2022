package frc.robot.AutoTools;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.Discretization;
import edu.wpi.first.math.system.LinearSystem;

/**
 * A Kalman filter combines predictions from a model and measurements to give an estimate
 * of the true system state. This is useful because many states cannot be measured
 * directly as a result of sensor noise, or because the state is "hidden".
 *
 * <p>
 * Kalman filters use a K gain matrix to determine whether to trust the model or
 * measurements more. Kalman filter theory uses statistics to compute an optimal K gain
 * which minimizes the sum of squares error in the state estimate. This K gain is used to
 * correct the state estimate by some amount of the difference between the actual
 * measurements and the measurements predicted by the model.
 *
 * <p>
 * This attempts to address some flaws present in the {@link KalmanFilter} class as the
 * KalmanFilter does not update the Kalman Gain dynamically.
 * 
 * @param <States>
 *            The number of columns in the system matrix
 * @param <Inputs>
 *            The number of rows in the input matrix
 * @param <Outputs>
 *            The number of rows in the output matrix
 * 
 * @author Spring Konstant 6357
 */
public class SKKalmanFilter<States extends Num, Inputs extends Num, Outputs extends Num>
{
    /** Identity Matrix (I)*/
    private Matrix<States, States> identity;

    private final Nat<States> states;

    /** System Matrix (A)*/
    private LinearSystem<States, Inputs, Outputs> plant;

    /** Output Matrix (C)*/
    private Matrix<Outputs, States> c;

    /** Kalman Gain (K)*/
    private Matrix<States, Outputs> k;

    /** Estimation of State (x̂)*/
    private Matrix<States, N1>  xHat;
    /** Corrected State (x)*/
    private Matrix<States, N1>  x;
    /** Innovation (ỹ)*/
    private Matrix<Outputs, N1> yTilda;

    /** Corrected Covariance (P)*/
    private Matrix<States, States>   p = new Matrix<>(new SimpleMatrix(new double[][]{{0.0, 0.0}, {0.0, 0.0}}));
    /** Covarience of Measurement (S)*/
    private Matrix<Outputs, Outputs> s;

    /** Adjustment to Covariance (Q)*/
    private Matrix<States, States>   contQ;
    /** Adjustment ot Measurement Covariance (R)*/
    private Matrix<Outputs, Outputs> contR;

    private Pair<Matrix<States, States>, Matrix<States, States>> pair;

    private Matrix<States, States>   discA;
    private Matrix<States, States>   discQ;
    private Matrix<Outputs, Outputs> discR;

    /**
     * Constructs a state-space observer with the given plant.
     *
     * @param states
     *            A Nat representing the states of the system.
     * @param outputs
     *            A Nat representing the outputs of the system.
     * @param plant
     *            The plant used for the prediction step.
     * @param stateStdDevs
     *            Standard deviations of model states.
     * @param measurementStdDevs
     *            Standard deviations of measurements.
     * @param dtSeconds
     *            Nominal discretization timestep.
     */
    public SKKalmanFilter(Nat<States> states, Nat<Outputs> outputs,
        LinearSystem<States, Inputs, Outputs> plant, Matrix<States, N1> stateStdDevs,
        Matrix<Outputs, N1> measurementStdDevs, double dtSeconds)
    {
        this.states = states;
        this.plant = plant;

        contQ = StateSpaceUtil.makeCovarianceMatrix(states, stateStdDevs);
        contR = StateSpaceUtil.makeCovarianceMatrix(outputs, measurementStdDevs);

        pair = Discretization.discretizeAQTaylor(plant.getA(), contQ, dtSeconds);
        discA = pair.getFirst();
        discQ = pair.getSecond();

        discR = Discretization.discretizeR(contR, dtSeconds);

        c = plant.getC();

        // P = APAᵀ + Q
        p = plant.getA().times(p).times(plant.getA().transpose()).plus(discQ);

        identity = Matrix.eye(states);

        reset();
    }

    /**
     * Sets x̂ to an empty matrix
     */
    public void reset()
    {
        xHat = new Matrix<>(states, Nat.N1());
    }

    /**
     * This simply guesses the state of the information based on the input. This will
     * eventually just increase the covariance unless updated using the update functions
     * of the Kalman Filter. The guess is made using the controller input and the time
     * since the last guess.
     * 
     * @param u New control input from controller.
     * @param dtSeconds Timestep for prediction.
     */
    public void predict(Matrix<Inputs, N1> u, double dtSeconds)
    {
        // Guessing the state of the system
        // x̂(n+1) = Ax(n) + Bu
        // TODO: I don't understand this black magic of a function
        xHat = plant.calculateX(x, u, dtSeconds);
        // Calculating Covariance
        // P̃ = AP(n)Aᵀ + Q
        p = plant.getA().times(p).times(plant.getA().transpose()).plus(contQ);
    }

    /**
     * Updates the value of x by looking at the value of y, covariance, and Kalman Gain.
     * This is also used to find the new covariance, Kalman Gain, and update the state
     * according to the information given by the second set of information. This should
     * ideally converge the error over time so as to not have the error increase and have
     * the value drift over time.
     * 
     * @param u Same control input used in the last predict step.
     * @param z Measurement vector.
     */
    public void correct(Matrix<Inputs, N1> u, Matrix<Outputs, N1> z)
    {
        // Calculating Residual
        // ỹ(n) = z(n) - Cx(n)
        yTilda = z.minus(c.times(x));

        // Calculating Measurement Covariance
        // S(n) = CP̃(n)Cᵀ + R
        s = c.times(p).times(c.transpose()).plus(contR);

        // Calculating Kalman Gain
        // K(n) = P(n)CᵀS⁻¹(n)
        k = p.times(c.transpose()).times(s.inv());

        // Updating the Corrected State
        // x(n+1) = x̂(n+1) + K(n)ỹ(n)
        x = xHat.plus(k.times(yTilda));

        // Updating Corrected Covariance
        // P(n+1) = (I - K(n)C)P̃(n+1)
        p = identity.minus(k.times(c)).times(p);

        // Updating Post Residual
        // ỹ(n+1) = z(n) - Cx(n+1)
        yTilda = z.minus(c.times(x));
    }

    /**
     * Returns the steady-state Kalman gain matrix K.
     *
     * @return The steady-state Kalman gain matrix K.
     */
    public Matrix<States, Outputs> getK()
    {
        return k;
    }

    /**
     * Returns an element of the steady-state Kalman gain matrix K.
     *
     * @param row
     *            Row of K.
     * @param col
     *            Column of K.
     * @return the element (i, j) of the steady-state Kalman gain matrix K.
     */
    public double getK(int row, int col)
    {
        return k.get(row, col);
    }

    /**
     * Set initial state estimate x̂.
     *
     * @param xhat
     *            The state estimate x̂.
     */
    public void setXhat(Matrix<States, N1> xhat)
    {
        this.xHat = xhat;
    }

    /**
     * Set an element of the initial state estimate x̂.
     *
     * @param row
     *            Row of x̂.
     * @param value
     *            Value for element of x̂.
     */
    public void setXhat(int row, double value)
    {
        xHat.set(row, 0, value);
    }

    /**
     * Returns the state estimate x̂.
     *
     * @return The state estimate x̂.
     */
    public Matrix<States, N1> getXhat()
    {
        return xHat;
    }

    /**
     * Returns an element of the state estimate x̂.
     *
     * @param row
     *            Row of x̂.
     * @return the state estimate x̂ at i.
     */
    public double getXhat(int row)
    {
        return xHat.get(row, 0);
    }
}
