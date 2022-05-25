#pragma once
//#include <ros/assert.h>
#include <iostream>
#include <Eigen\Dense>

#include "utility.h"
#include "parameters.h"
//#include "integration_base.h"

#include <ceres/ceres.h>


class IMUFactor : public ceres::SizedCostFunction<15, 7, 9, 7, 9>
{
  public:
    IMUFactor() = delete;
    IMUFactor(Eigen::Vector3d dp_,Eigen::Quaterniond dq_,Eigen::Vector3d dv_,Eigen::Vector3d ba_,Eigen::Vector3d bg_,
            Eigen::Matrix<double, 15, 15> jacobian_,Eigen::Matrix<double, 15, 15> covariance_,double dt_)
            :dp(dp_),dq(dq_),dv(dv_),ba(ba_),bg(bg_),jacobian(jacobian_),covariance(covariance_),dt(dt_)
    {
    }
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {

        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

        Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Vector3d Bai(parameters[1][3], parameters[1][4], parameters[1][5]);
        Eigen::Vector3d Bgi(parameters[1][6], parameters[1][7], parameters[1][8]);

        Eigen::Vector3d Pj(parameters[2][0], parameters[2][1], parameters[2][2]);
        Eigen::Quaterniond Qj(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

        Eigen::Vector3d Vj(parameters[3][0], parameters[3][1], parameters[3][2]);
        Eigen::Vector3d Baj(parameters[3][3], parameters[3][4], parameters[3][5]);
        Eigen::Vector3d Bgj(parameters[3][6], parameters[3][7], parameters[3][8]);

//Eigen::Matrix<double, 15, 15> Fd;
//Eigen::Matrix<double, 15, 12> Gd;

//Eigen::Vector3d pPj = Pi + Vi * sum_t - 0.5 * g * sum_t * sum_t + corrected_delta_p;
//Eigen::Quaterniond pQj = Qi * delta_q;
//Eigen::Vector3d pVj = Vi - g * sum_t + corrected_delta_v;
//Eigen::Vector3d pBaj = Bai;
//Eigen::Vector3d pBgj = Bgi;

//Vi + Qi * delta_v - g * sum_dt = Vj;
//Qi * delta_q = Qj;

//delta_p = Qi.inverse() * (0.5 * g * sum_dt * sum_dt + Pj - Pi);
//delta_v = Qi.inverse() * (g * sum_dt + Vj - Vi);
//delta_q = Qi.inverse() * Qj;

#if 0
        if ((Bai - ba).norm() > 0.10 ||
            (Bgi - bg).norm() > 0.01)
        {
            pre_integration->repropagate(Bai, Bgi);
        }
#endif

        Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);
//         residual = pre_integration->evaluate(Pi, Qi, Vi, Bai, Bgi,
//                                             Pj, Qj, Vj, Baj, Bgj);
        Eigen::Matrix<double, 15, 1> temp_residuals;
        Eigen::Matrix3d dp_dba = jacobian.block<3, 3>(O_P, O_BA);// derivatives dp/dba, position/acc bias
        Eigen::Matrix3d dp_dbg = jacobian.block<3, 3>(O_P, O_BG);

        Eigen::Matrix3d dq_dbg = jacobian.block<3, 3>(O_R, O_BG);

        Eigen::Matrix3d dv_dba = jacobian.block<3, 3>(O_V, O_BA);
        Eigen::Matrix3d dv_dbg = jacobian.block<3, 3>(O_V, O_BG);

        Eigen::Vector3d dba = Bai - ba;
        Eigen::Vector3d dbg = Bgi - bg;
        // 三个预积分变量通过bias更新
        //std::cout<<"Bai:"<<Bai<<std::endl<<"Bgi:"<<Bgi<<std::endl;
        //std::cout<<"dv:"<<dv<<std::endl<<dv_dba<<std::endl<<dba<<std::endl<<dv_dbg<<std::endl<<Bgi<<std::endl<<bg<<std::endl;
        Eigen::Quaterniond corrected_delta_q = dq * Utility::deltaQ(dq_dbg * dbg);//corrected_delta_q is q_bibj
        Eigen::Vector3d corrected_delta_v = dv + dv_dba * dba + dv_dbg * dbg;//corrected_delta_v is beta_bibj
        Eigen::Vector3d corrected_delta_p = dp + dp_dba * dba + dp_dbg * dbg;//corrected_delta_p is alpha_bibj
        //std::cout<<"corrected dv:"<<corrected_delta_v<<std::endl;
        temp_residuals.block<3, 1>(O_P, 0) = Qi.inverse() * (0.5 * G * dt * dt + Pj - Pi - Vi * dt) - corrected_delta_p;
        temp_residuals.block<3, 1>(O_R, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
        temp_residuals.block<3, 1>(O_V, 0) = Qi.inverse() * (G * dt + Vj - Vi) - corrected_delta_v;
        temp_residuals.block<3, 1>(O_BA, 0) = Baj - Bai;
        temp_residuals.block<3, 1>(O_BG, 0) = Bgj - Bgi;
        //std::cout<<"state dv:"<<Qi.inverse() * (G * dt + Vj - Vi)<<std::endl;
        residual = temp_residuals;
        //std::cout<<1<<std::endl<<residual<<std::endl<<2<<std::endl;
        Eigen::Matrix<double, 15, 15> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 15, 15>>(covariance.inverse()).matrixL().transpose();
        //sqrt_info.setIdentity();
        residual = sqrt_info * residual;
        //std::cout<<residual<<std::endl;
        if (jacobians)//求r关于状态变量x的J，r是15维，x分了四块，维度分别是7 9 7 9，J应该是要给ceres
        {
            double sum_dt = dt;
            Eigen::Matrix3d dp_dba = jacobian.template block<3, 3>(O_P, O_BA);
            Eigen::Matrix3d dp_dbg = jacobian.template block<3, 3>(O_P, O_BG);

            Eigen::Matrix3d dq_dbg = jacobian.template block<3, 3>(O_R, O_BG);

            Eigen::Matrix3d dv_dba = jacobian.template block<3, 3>(O_V, O_BA);
            Eigen::Matrix3d dv_dbg = jacobian.template block<3, 3>(O_V, O_BG);

            if (jacobian.maxCoeff() > 1e8 || jacobian.minCoeff() < -1e8)
            {
                std::cout<<"numerical unstable in preintegration"<<std::endl;
                //std::cout << jacobian << std::endl;
///                ROS_BREAK();
            }

            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
                jacobian_pose_i.setZero();

                jacobian_pose_i.block<3, 3>(O_P, O_P) = -Qi.inverse().toRotationMatrix();
                jacobian_pose_i.block<3, 3>(O_P, O_R) = Utility::skewSymmetric(Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));

#if 0
            jacobian_pose_i.block<3, 3>(O_R, O_R) = -(Qj.inverse() * Qi).toRotationMatrix();
#else
                Eigen::Quaterniond corrected_delta_q = dq * Utility::deltaQ(dq_dbg * (Bgi - bg));
                jacobian_pose_i.block<3, 3>(O_R, O_R) = -(Utility::Qleft(Qj.inverse() * Qi) * Utility::Qright(corrected_delta_q)).bottomRightCorner<3, 3>();
#endif

                jacobian_pose_i.block<3, 3>(O_V, O_R) = Utility::skewSymmetric(Qi.inverse() * (G * sum_dt + Vj - Vi));

                jacobian_pose_i = sqrt_info * jacobian_pose_i;

                if (jacobian_pose_i.maxCoeff() > 1e8 || jacobian_pose_i.minCoeff() < -1e8)
                {
                   std::cout<<"numerical unstable in preintegration"<<std::endl;
                    //std::cout << sqrt_info << std::endl;
                    //ROS_BREAK();
                }
            }
            if (jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_speedbias_i(jacobians[1]);
                jacobian_speedbias_i.setZero();
                jacobian_speedbias_i.block<3, 3>(O_P, O_V - O_V) = -Qi.inverse().toRotationMatrix() * sum_dt;
                jacobian_speedbias_i.block<3, 3>(O_P, O_BA - O_V) = -dp_dba;
                jacobian_speedbias_i.block<3, 3>(O_P, O_BG - O_V) = -dp_dbg;

#if 0
            jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -dq_dbg;
#else
                //Eigen::Quaterniond corrected_delta_q = dq * Utility::deltaQ(dq_dbg * (Bgi - bg));
                //jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -Utility::Qleft(Qj.inverse() * Qi * corrected_delta_q).bottomRightCorner<3, 3>() * dq_dbg;
                jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -Utility::Qleft(Qj.inverse() * Qi * dq).bottomRightCorner<3, 3>() * dq_dbg;
#endif

                jacobian_speedbias_i.block<3, 3>(O_V, O_V - O_V) = -Qi.inverse().toRotationMatrix();
                jacobian_speedbias_i.block<3, 3>(O_V, O_BA - O_V) = -dv_dba;
                jacobian_speedbias_i.block<3, 3>(O_V, O_BG - O_V) = -dv_dbg;

                jacobian_speedbias_i.block<3, 3>(O_BA, O_BA - O_V) = -Eigen::Matrix3d::Identity();

                jacobian_speedbias_i.block<3, 3>(O_BG, O_BG - O_V) = -Eigen::Matrix3d::Identity();

                jacobian_speedbias_i = sqrt_info * jacobian_speedbias_i;

                //ROS_ASSERT(fabs(jacobian_speedbias_i.maxCoeff()) < 1e8);
                //ROS_ASSERT(fabs(jacobian_speedbias_i.minCoeff()) < 1e8);
            }
            if (jacobians[2])
            {
                Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[2]);
                jacobian_pose_j.setZero();

                jacobian_pose_j.block<3, 3>(O_P, O_P) = Qi.inverse().toRotationMatrix();

#if 0
            jacobian_pose_j.block<3, 3>(O_R, O_R) = Eigen::Matrix3d::Identity();
#else
                Eigen::Quaterniond corrected_delta_q = dq * Utility::deltaQ(dq_dbg * (Bgi - bg));
                jacobian_pose_j.block<3, 3>(O_R, O_R) = Utility::Qleft(corrected_delta_q.inverse() * Qi.inverse() * Qj).bottomRightCorner<3, 3>();
#endif

                jacobian_pose_j = sqrt_info * jacobian_pose_j;

                //ROS_ASSERT(fabs(jacobian_pose_j.maxCoeff()) < 1e8);
                //ROS_ASSERT(fabs(jacobian_pose_j.minCoeff()) < 1e8);
            }
            if (jacobians[3])
            {
                Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_speedbias_j(jacobians[3]);
                jacobian_speedbias_j.setZero();

                jacobian_speedbias_j.block<3, 3>(O_V, O_V - O_V) = Qi.inverse().toRotationMatrix();

                jacobian_speedbias_j.block<3, 3>(O_BA, O_BA - O_V) = Eigen::Matrix3d::Identity();

                jacobian_speedbias_j.block<3, 3>(O_BG, O_BG - O_V) = Eigen::Matrix3d::Identity();

                jacobian_speedbias_j = sqrt_info * jacobian_speedbias_j;

                //ROS_ASSERT(fabs(jacobian_speedbias_j.maxCoeff()) < 1e8);
                //ROS_ASSERT(fabs(jacobian_speedbias_j.minCoeff()) < 1e8);
            }
        }

        return true;
    }

    //bool Evaluate_Direct(double const *const *parameters, Eigen::Matrix<double, 15, 1> &residuals, Eigen::Matrix<double, 15, 30> &jacobians);

    //void checkCorrection();
    //void checkTransition();
    //void checkJacobian(double **parameters);
    //IntegrationBase* pre_integration;
    
    Eigen::Vector3d dp;
    Eigen::Quaterniond dq;
    Eigen::Vector3d dv;
    Eigen::Vector3d ba;
    Eigen::Vector3d bg;
    Eigen::Matrix<double, 15, 15> jacobian;
    Eigen::Matrix<double, 15, 15> covariance;
    double dt;
};
