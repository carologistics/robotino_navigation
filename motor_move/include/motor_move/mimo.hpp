#include <eigen3/Eigen/Dense>

class MIMO_PID {
    public:
        MIMO_PID(const Eigen::MatrixXd Kp, const Eigen::MatrixXd Ki, const Eigen::MatrixXd Kd);
        MIMO_PID();
        ~MIMO_PID();
        Eigen::MatrixXd compute(const Eigen::MatrixXd error, const double dt);
        void set_Kp(const Eigen::MatrixXd Kp);
        void set_Ki(const Eigen::MatrixXd Ki);
        void set_Kd(const Eigen::MatrixXd Kd);
    private:
        Eigen::MatrixXd Kp;
        Eigen::MatrixXd Ki;
        Eigen::MatrixXd Kd;
        Eigen::MatrixXd integral;
        Eigen::MatrixXd derivative;
        Eigen::MatrixXd error_prev;
};
