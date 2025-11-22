#ifndef EIGEN_MODEL_H
#define EIGEN_MODEL_H
    #include <ArduinoEigenDense.h>
    using namespace std;
    using namespace Eigen;

class StateSpaceModel {
    public:

        // System dimensions (default)
        int n = 4;          // Number of states
        int m = 1;          // Number of inputs
        int p = 4;          // Number of outputs
        float Ts = 0.01;    // Sampling time

        MatrixXf C, A_d, B_d, Kf, I;                // State matricies
        MatrixXf Q, R, P, P_prev, P_pred, S;        // Covariances
        VectorXf x, x_prev, x_pred, u_prev, v;      // Inputs and outputs
        VectorXf x_ref;                             // Control parameters
        RowVectorXf K_lqr;

        StateSpaceModel() {
            C.resize(p, n);
            A_d.resize(n, n);
            B_d.resize(n, m);
            Kf.resize(n, p);
            I = MatrixXf::Identity(n, n);

            Q.resize(n, n);
            R.resize(p, p);
            P.resize(n, n);
            P_prev.resize(n, n);
            P_pred.resize(n, n);

            x.resize(n);
            x_prev.resize(n);
            x_pred.resize(n);
            u_prev.resize(m);
            v.resize(p);
            S.resize(p, p);

            x_ref.resize(n);
            K_lqr.resize(1, n);
        }

        VectorXf kalmanFilter(const VectorXf& y_meas){
            // Called each sampling interval Ts
            // Prediction
            x_pred = A_d*x_prev + B_d*u_prev;
            P_pred = A_d*P_prev*A_d.transpose() + Q;

            // Measurement update
            S = C*P_pred*C.transpose() + R;
            Kf = P_pred*C.transpose()*S.inverse();
            v = y_meas - C*x_pred;
            
            // Update filter estimate
            x = x_pred + Kf*v;                       // Corrected state estimate
            P = P_pred - Kf*S*Kf.transpose();       // Reduced P_pred with measurement

            // for next iteration
            x_prev = x;
            P_prev = P;

            return x;
        }
};

StateSpaceModel Model;

#endif // EIGEN_MODEL_H