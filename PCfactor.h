#pragma once
//#include <ros/assert.h>
#include <iostream>
#include <Eigen\Dense>

#include "utility.h"
//#include "parameters.h"
//#include "integration_base.h"

#include <ceres/ceres.h>

struct PCfactor
{
	PCfactor(Eigen::Vector3d deltaP_, Eigen::Quaterniond deltaQ_, double noise_t_, double noise_q_)
		: deltaP(deltaP_), deltaQ(deltaQ_),noise_t(noise_t_),noise_q(noise_q_) {}

	template <typename T>
	bool operator()(const T *Pose1, const T *Pose2, T *residual) const
	{
        Eigen::Matrix<T, 3, 1> P1(Pose1[0], Pose1[1], Pose1[2]);
        Eigen::Quaternion<T> Q1(Pose1[6], Pose1[3], Pose1[4], Pose1[5]);
        Eigen::Matrix<T, 3, 1> P2(Pose2[0], Pose2[1], Pose2[2]);
        Eigen::Quaternion<T> Q2(Pose2[6], Pose2[3], Pose2[4], Pose2[5]);
        
        Eigen::Matrix<T, 3, 1> dp(T(deltaP.x()), T(deltaP.y()), T(deltaP.z()));
        Eigen::Quaternion<T> dq(T(deltaQ.w()), T(deltaQ.x()), T(deltaQ.y()), T(deltaQ.z()));
        
        Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residual);
        residuals.block<3, 1>(0, 0) = Q1.inverse() * (P2 - P1) - dp;
        residuals.block<3, 1>(3, 0) = (dq.inverse() * (Q1.inverse() * Q2)).vec();
        Eigen::Matrix<T,6,6> sqrt_info;
        sqrt_info << T(noise_t),T(0),T(0),T(0),T(0),T(0),
                     T(0),T(noise_t),T(0),T(0),T(0),T(0),
                     T(0),T(0),T(noise_t),T(0),T(0),T(0),
                     T(0),T(0),T(0),T(noise_q),T(0),T(0),
                     T(0),T(0),T(0),T(0),T(noise_q),T(0),
                     T(0),T(0),T(0),T(0),T(0),T(noise_q);
        //cout<<noise_t<<endl<<noise_q;
        //sqrt_info.setIdentity();
        residuals = sqrt_info.inverse()*residuals;        
		return true;
	}

	static ceres::CostFunction *Create(const Eigen::Vector3d deltaP_, const Eigen::Quaterniond deltaQ_ , double noise_t_,double noise_q_)
	{
		return (new ceres::AutoDiffCostFunction<PCfactor, 6, 7, 7>(new PCfactor(deltaP_, deltaQ_,noise_t_,noise_q_)));
	}

	Eigen::Vector3d deltaP;
    Eigen::Quaterniond deltaQ;
    double noise_t;
    double noise_q;
//     Eigen::Vector3d P1;
//     Eigen::Quaterniond Q1;
//     Eigen::Vector3d P2;
//     Eigen::Quaterniond Q2;
};