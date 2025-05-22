#ifndef LINEAR_QUADRATIC_H
    #define LINEAR_QUADRATIC_H
        #include <ArduinoEigenDense.h>
        using namespace std;
        using namespace Eigen;
#endif

class EigenLQR {
    public:
        // System dimensions (default)
        int n = 4;          // Number of states A_d.rows(),
        int m = 2;          // Number of inputs B_d.cols(),
        float Ts = 0.01;    // Sampling time

        // LQR Matrices
        MatrixXf Q = MatrixXf::Zero(n, n);            // State weight matrix
        MatrixXf R = MatrixXf::Zero(m, m);            // Input weight matrix
        MatrixXf P = MatrixXf::Identity(n,n);                             // Riccati solution with inital guess
        MatrixXf L = MatrixXf::Zero(m, n);            // LQR gain
        VectorXf x_ref = VectorXf::Zero(n);           // Reference state

        void init(const MatrixXf& A_d, const MatrixXf& B_d) {
            // A_d and B_d are discretized state space matricies
            // In some cases the Riccati solution might not converge,
            // The LQR gain L will be zero, preventing state based input.

            // Solve Discrete-time Algebraic Riccati Equation (DARE)
            MatrixXf P_prev = MatrixXf::Zero(n, n);
            float tolerance = 1;
            int max_iterations = 1000;
            int i = 0;

            while ((P - P_prev).norm() > tolerance && i <= max_iterations) {
                P_prev = P;

                MatrixXf K = P*B_d*(R + B_d.transpose()*P*B_d).inverse();
                P = Q + A_d.transpose()*(P - K*B_d.transpose()*P)*A_d;

                if (i == max_iterations){
                    Serial.println("The Riccati matrix P has not converged!");
                }
                i++;
            };
            // Compute LQR Gain: L = (R + B^T P B)^-1 B^T P A
            L = (R + B_d.transpose()*P*B_d).inverse()*B_d.transpose()*P*A_d;
        }
};