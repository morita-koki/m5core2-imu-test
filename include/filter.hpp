#ifndef _FILETER_HPP_
#define _FILETER_HPP_

#include <ArduinoEigen.h>
#include <tuple>
// #include <cmath>

#define RAD2DEG(rad) ((rad) * 180.0 / M_PI)
#define DEG2RAD(deg) ((deg) * M_PI / 180.0)

using namespace std;
using namespace Eigen;


class KalmanFilter {
private:

public:
    KalmanFilter(){}
    ~KalmanFilter(){}

    tuple<MatrixXd, MatrixXd> update(const MatrixXd& x, const MatrixXd& u, const MatrixXd& z, const MatrixXd& P, const MatrixXd& R, const MatrixXd& Q)
    {
        // prediction
        MatrixXd x_pred = predict_x(x, u);
        MatrixXd F = calc_F(x, u);
        MatrixXd P_pred = predict_P(P, F, Q);
        MatrixXd H = calc_H(x_pred);

        // update
        MatrixXd y_res = update_y_res(z, x_pred);
        MatrixXd S = update_S(P_pred, H, R);
        MatrixXd K = update_K(P_pred, H, S);
        MatrixXd x_upd = update_x(x_pred, K, y_res);
        MatrixXd P_upd = update_P(P_pred, K, H);

        if (x_upd(2,0) > 180) x_upd(2,0) -= 180;
        if (x_upd(2,0) < -180) x_upd(2,0) += 180;
        return make_tuple(x_upd, P_upd);
        // return make_tuple(x, P);
    }
    

private:
    /*
    def f(x, u):
    u_x, u_y, u_z = u[0][0], u[1][0], u[2][0]
    c1, s1 = np.cos(x[0][0]), np.sin(x[0][0])
    c2, s2 = np.cos(x[1][0]), np.sin(x[1][0])
    c3, s3 = np.cos(x[2][0]), np.sin(x[2][0])
    x = np.array([
        [x[0][0]+u_x+u_y*s1*s2/c2+u_z*c1*s2/c2],
        [x[1][0]+u_y*c1-u_z*s1],
        [x[2][0]+u_y*s1/c2+u_z*c1/c2]
    ])
    return x
    */
    MatrixXd f(const MatrixXd& x, const MatrixXd& u) 
    {
        MatrixXd x_ret(3,1);
        float u_x = u(0,0), u_y = u(1,0), u_z = u(2,0);
        float c1 = cos(DEG2RAD(x(0,0))), s1 = sin(DEG2RAD(x(0,0)));
        float c2 = cos(DEG2RAD(x(1,0))), s2 = sin(DEG2RAD(x(1,0)));
        float c3 = cos(DEG2RAD(x(2,0))),  s3 = sin(DEG2RAD(x(2,0)));
        x_ret(0,0) = x(0,0) + u_x + u_y*s1*s2/c2 + u_z*c1*s2/c2;
        x_ret(1,0) = x(1,0) + u_y*c1 - u_z*s1;
        x_ret(2,0) = x(2,0) + u_y*s1/c2 + u_z*c1/c2;
        return x_ret;
        // MatrixXd x_ret(3,1);

        // x_ret(0,0) = x(0,0) + u(0,0) + u(1,0)*sin(x(0,0))*tan(x(1,0)) + u(2,0)*cos(x(0,0))*tan(x(1,0));
        // x_ret(1,0) = x(1,0) + u(1,0)*cos(x(0,0)) - u(2,0)*sin(x(0,0));
        // x_ret(2,0) = x(2,0) + u(1,0)*sin(x(0,0))/cos(x(1,0)) + u(2,0)*cos(x(0,0))/cos(x(1,0));

        // return x_ret;
    }

    MatrixXd h(const MatrixXd& x)
    {
        MatrixXd h(2, 3);
        h << 1, 0, 0,
             0, 1, 0;
        
        MatrixXd y = h * x;
        return y;
    }

    MatrixXd predict_x(const MatrixXd& x, const MatrixXd& u)
    {
        return f(x, u);
    }

    MatrixXd predict_P(const MatrixXd& P, const MatrixXd& F, const MatrixXd& Q)
    {
        return F * P * F.transpose() + Q;
    }

    MatrixXd calc_F(const MatrixXd& x, const MatrixXd& u)
    {
        float u_x = u(0,0), u_y = u(1,0), u_z = u(2,0);
        float c1 = cos(DEG2RAD(x(0,0))), s1 = sin(DEG2RAD(x(0,0)));
        float c2 = cos(DEG2RAD(x(1,0))), s2 = sin(DEG2RAD(x(1,0)));
        float c3 = cos(DEG2RAD(x(2,0))),  s3 = sin(DEG2RAD(x(2,0)));

        MatrixXd F_ret(3, 3);
        F_ret << 1+u_y*c1*s2/c2-u_z*s1*s2/c2, u_y*s1/pow(c2,2)+u_z*c1/pow(c2, 2), 0,
             -u_y*s1-u_z*c1, 1, 0,
             u_y*c1/c2-u_z*s1/c2, u_y*s1*s2/pow(c2,2)+u_z*c1*s2/pow(c2,2), 1;
        
        return F_ret;
    }
    
    MatrixXd calc_H(const MatrixXd& x)
    {
        MatrixXd H(2, 3);
        H << 1, 0, 0,
             0, 1, 0;
        return H;
    }

    MatrixXd update_y_res(const MatrixXd& z, const MatrixXd& x)
    {
        MatrixXd y_res = z - h(x);
        return y_res;
    }

    MatrixXd update_S(const MatrixXd& P, const MatrixXd& H, const MatrixXd& R)
    {
        return H * P * H.transpose() + R;
    }

    MatrixXd update_K(const MatrixXd& P, const MatrixXd& H, const MatrixXd& S)
    {
        return P * H.transpose() * S.inverse();
    }

    MatrixXd update_x(const MatrixXd& x, const MatrixXd& K, const MatrixXd& y_res)
    {
        return x + K * y_res;
    }

    MatrixXd update_P(const MatrixXd& P, const MatrixXd& K, const MatrixXd& H)
    {
        return (MatrixXd::Identity(3,3) - K * H) * P;
    }


};

#endif