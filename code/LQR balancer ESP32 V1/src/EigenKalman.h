#ifndef EIGEN_KALMAN_H
    #define EIGEN_KALMAN_H
        #include <ArduinoEigenDense.h>
        using namespace std;
        using namespace Eigen;
#endif

class EigenKalmanFilter {
    public:
        // System dimensions (default)
        int n = 4;          // Number of states
        int m = 2;          // Number of inputs
        int p = 4;          // Number of outputs
        float Ts = 0.01;    // Sampling time

        // Kalman state matricies
        MatrixXf A = MatrixXf::Zero(n, n);            // Transition matrix
        MatrixXf B = MatrixXf::Zero(n, m);            // Input matrix
        MatrixXf C = MatrixXf::Zero(p, n);            // Measurement model
        MatrixXf A_d = MatrixXf::Zero(n, n);          // Discretized transition matrix
        MatrixXf B_d = MatrixXf::Zero(n, m);          // Discretized input matrix
        MatrixXf K = MatrixXf::Zero(n, p);            // Kalman gain
        MatrixXf I = MatrixXf::Identity(n, n);

        // Covariances
        MatrixXf Q = MatrixXf::Zero(n, n);            // Process noise covariance
        MatrixXf R = MatrixXf::Zero(p, p);            // Measurement noise covariance
        MatrixXf P = MatrixXf::Zero(n, n);            // Estimation error covariance
        MatrixXf P_pred = MatrixXf::Zero(n, n);       // Predicted estimation error covariance    

        VectorXf x_prior = VectorXf::Zero(n);         // State prior/last estimated state x_k-1/k-1 
        VectorXf x_pred = VectorXf::Zero(n);          // Predicted state x_k/k-1
        VectorXf x = VectorXf::Zero(n);               // State estimate/ posterior x_k/k
        VectorXf u_prev = VectorXf::Zero(m);          // Previous input
        VectorXf v = VectorXf::Zero(p);               // Innovation
        MatrixXf S = MatrixXf::Zero(p, p);            // Innovation covariance


        void discretize_state_matricies(){
            // Discretization (Taylor series)
            MatrixXf Psi = MatrixXf::Zero(n,n);
            
            Psi = I * Ts + (A*(pow(Ts,2) / 2)) + (A*A*(pow(Ts,3) / 6)) + (A*A*A*(pow(Ts,4) / 24)) + (A*A*A*A*(pow(Ts,5) / 120));
            A_d = I + A * Psi;
            B_d = Psi * B;
        }

        void kalman_filter(VectorXf y){
            // Called each sampling interval Ts
            // Predicts the next state from prior states and previous input u
            x_pred = A_d*x_prior + B_d*u_prev;
            P_pred = A_d*P*A_d.transpose() + Q;

            // Update state with measurement y
            v = y - C*x_pred;
            S = C*P_pred*C.transpose() + R;
            K = P_pred*C.transpose()*S.inverse();
            P = P_pred - K*S*K.transpose();         // Reduced P_pred with measurement

            x = x_pred + K*v;                       // Corrected state estimate

            // for next iteration
            x_prior = x;
        }
};