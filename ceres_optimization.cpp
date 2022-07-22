// mex
#pragma once

#include "mex.h"

#include <iostream>
//#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>
#include <Eigen\Dense>

#include "pose_local_parameterization.h"
#include "IMUfactor.h"
#include "PCfactor.h"

using namespace std;

// 代价函数的计算模型

const int WINDOW_SIZE = 10;
const int SIZE_POSE = 7;
const int SIZE_SPEEDBIAS = 9;
        
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) 
{
  cout<<"number of input parameters:"<<nrhs<<"\n";
  
# if 0
  mwSize ninput_dimension0 = mxGetNumberOfDimensions(prhs[2]);
  const mwSize * input_dimension0 = mxGetDimensions(prhs[2]);
  cout<<"number of dimensions of fisrt input paramters:"<< ninput_dimension0<<"\n";
  for(mwSize i=0;i<ninput_dimension0;i++){
        cout<<input_dimension0[i]<<"\n";}
  //cout<<"size of fisrt input paramters:"<< input_dimension0<<"\n";
  double* inMatrix1;
  inMatrix1 = mxGetPr(prhs[0]);
  double Ps[3][11];
  memcpy(Ps, inMatrix1, sizeof(Ps));
  
  // cout<<inMatrix1[1][5]<<"\n"; 报错 inMatrix1 是指针不是数组
  cout<<Ps[1][5]<<"\n"<<sizeof(Ps)/sizeof(Ps[0])<<sizeof(Ps[0])/sizeof(Ps[0][0])<<"\n";
# endif  

  // check if Ps stores corretc values
# if 0
  double* inMatrix0;
  inMatrix0 = mxGetPr(prhs[2]);
  double Ps[11][3];
  memcpy(Ps, inMatrix0, sizeof(Ps));
  //cout<<Ps[0][0]<<" "<<Ps[0][1]<<" "<<Ps[0][2]<<" "<<Ps[0][3]<<" "<<Ps[0][4]<<" "<<Ps[0][5]<<" ";
  for(mwSize i=0;i<11;i++){
      for (mwSize j=0;j<3;j++){
        cout<<Ps[i][j]<<" ";
        if(j==2){
            cout<<"\n";
        }
      }
  }
# endif
  
  // check if Qs stores corretc values
# if 0
  double* inMatrix2;
  inMatrix2 = mxGetPr(prhs[2]);
  double Qs[11][4];
  memcpy(Qs, inMatrix2, sizeof(Qs));
  //cout<<Ps[0][0]<<" "<<Ps[0][1]<<" "<<Ps[0][2]<<" "<<Ps[0][3]<<" "<<Ps[0][4]<<" "<<Ps[0][5]<<" ";
  for(mwSize i=0;i<11;i++){
      for (mwSize j=0;j<4;j++){
        cout<<Qs[i][j]<<" ";
        if(j==3){
            cout<<"\n";
        }
      }
  }
# endif 

  // check if Vs stores corretc values
# if 0
  double* inMatrix3;
  inMatrix3 = mxGetPr(prhs[3]);
  double Vs[11][3];
  memcpy(Vs, inMatrix3, sizeof(Vs));
  //cout<<Ps[0][0]<<" "<<Ps[0][1]<<" "<<Ps[0][2]<<" "<<Ps[0][3]<<" "<<Ps[0][4]<<" "<<Ps[0][5]<<" ";
  for(mwSize i=0;i<11;i++){
      for (mwSize j=0;j<3;j++){
        cout<<Vs[i][j]<<" ";
        if(j==2){
            cout<<"\n";
        }
      }
  }
# endif  

  // check if Bas stores corretc values
# if 0
  double* inMatrix4;
  inMatrix4 = mxGetPr(prhs[4]);
  double Bas[11][3];
  memcpy(Bas, inMatrix4, sizeof(Bas));
  //cout<<Ps[0][0]<<" "<<Ps[0][1]<<" "<<Ps[0][2]<<" "<<Ps[0][3]<<" "<<Ps[0][4]<<" "<<Ps[0][5]<<" ";
  for(mwSize i=0;i<11;i++){
      for (mwSize j=0;j<3;j++){
        cout<<Bas[i][j]<<" ";
        if(j==2){
            cout<<"\n";
        }
      }
  }
# endif  
 
   // check if dpContainer stores corretc values
# if 0
  double* inMatrix6;
  inMatrix6 = mxGetPr(prhs[6]);
  double dpContainer[10][3];
  memcpy(dpContainer, inMatrix6, sizeof(dpContainer));
  //cout<<Ps[0][0]<<" "<<Ps[0][1]<<" "<<Ps[0][2]<<" "<<Ps[0][3]<<" "<<Ps[0][4]<<" "<<Ps[0][5]<<" ";
  for(mwSize i=0;i<10;i++){
      for (mwSize j=0;j<3;j++){
        cout<<dpContainer[i][j]<<" ";
        if(j==2){
            cout<<"\n";
        }
      }
  }
# endif 
  
   // check if dqContainer stores corretc values
# if 0
  double* inMatrix7;
  inMatrix7 = mxGetPr(prhs[7]);
  double dqContainer[10][4];
  memcpy(dqContainer, inMatrix7, sizeof(dqContainer));
  //cout<<Ps[0][0]<<" "<<Ps[0][1]<<" "<<Ps[0][2]<<" "<<Ps[0][3]<<" "<<Ps[0][4]<<" "<<Ps[0][5]<<" ";
  for(mwSize i=0;i<10;i++){
      for (mwSize j=0;j<4;j++){
        cout<<dqContainer[i][j]<<" ";
        if(j==3){
            cout<<"\n";
        }
      }
  }
  Eigen::Quaterniond test1;
  test1=Eigen::Quaterniond(dqContainer[0][0],dqContainer[0][1],dqContainer[0][2],dqContainer[0][3]);
  cout<<test1.w()<<endl<<test1.vec()<<endl;
# endif 
  
# if 0
  mwSize ninput_dimension0 = mxGetNumberOfDimensions(prhs[16]);
  const mwSize * input_dimension0 = mxGetDimensions(prhs[16]);
  cout<<"number of dimensions of fisrt input paramters:"<< ninput_dimension0<<"\n";
  for(mwSize i=0;i<ninput_dimension0;i++){
        cout<<input_dimension0[i]<<"\n";}

  double* inMatrix16;
  inMatrix16 = mxGetPr(prhs[16]);
  int frame_count;
  frame_count = *inMatrix16;

  cout<<"frame count:"<<frame_count<<"\n";
# endif 

# if 0
  mwSize ninput_dimension15 = mxGetNumberOfDimensions(prhs[14]);
  const mwSize * input_dimension15 = mxGetDimensions(prhs[14]);
  cout<<"number of dimensions of 15th input paramters:"<< ninput_dimension15<<"\n";
  for(mwSize i=0;i<ninput_dimension15;i++){
        cout<<input_dimension15[i]<<"\n";}
  //cout<<"size of fisrt input paramters:"<< input_dimension0<<"\n";
  double* inMatrix14;
  inMatrix14 = mxGetPr(prhs[14]);
  double icp2last[WINDOW_SIZE][7];
  memcpy(icp2last, inMatrix14, sizeof(icp2last));
  for(mwSize i=0;i<frame_count-1;i++){
      for (mwSize j=0;j<7;j++){
          cout<<icp2last[i][j]<<" ";
          if(j==6){
              cout<<"\n";
          }
      }
  }
# endif
  
# if 0
  mwSize ninput_dimension16 = mxGetNumberOfDimensions(prhs[15]);
  const mwSize * input_dimension16 = mxGetDimensions(prhs[15]);
  cout<<"number of dimensions of fisrt input paramters:"<< ninput_dimension16<<"\n";
  for(mwSize i=0;i<ninput_dimension16;i++){
        cout<<input_dimension16[i]<<"\n";}
  //cout<<"size of fisrt input paramters:"<< input_dimension0<<"\n";
  double* inMatrix15;
  inMatrix15 = mxGetPr(prhs[15]);
  double icp2base[7];
  // icp2base = *inMatrix15;
  memcpy(icp2base, inMatrix15, sizeof(icp2base));
  for(mwSize i=0;i<7;i++){
      cout<<icp2base[i]<<" ";
      cout<<"\n";
  }
# endif    
  
# if 0
  mwSize ninput_dimension17 = mxGetNumberOfDimensions(prhs[17]);
  const mwSize * input_dimension17 = mxGetDimensions(prhs[17]);
  cout<<"number of dimensions of 18th input paramters:"<< ninput_dimension17<<"\n";
  for(mwSize i=0;i<ninput_dimension17;i++){
        cout<<input_dimension17[i]<<"\n";}
  //cout<<"size of fisrt input paramters:"<< input_dimension0<<"\n";
  double* inMatrix17;
  inMatrix17 = mxGetPr(prhs[17]);
  double g_sum[WINDOW_SIZE+1][3];
  memcpy(g_sum, inMatrix17, sizeof(g_sum));
  for(mwSize i=0;i<frame_count;i++){
      for (mwSize j=0;j<3;j++){
          cout<<g_sum[i][j]<<" ";
          if(j==2){
              cout<<"\n";
          }
      }
  }
# endif

// main function  
# if 1  
  // first,get the frame count
  double* inMatrix16;
  inMatrix16 = mxGetPr(prhs[16]);
  int frame_count;
  frame_count = *inMatrix16;
  
  double para_Pose[WINDOW_SIZE+1][SIZE_POSE];
  double para_SpeedBias[WINDOW_SIZE+1][SIZE_SPEEDBIAS];
  
  double Ps[11][3],Rs[3][3][11],Qs[11][4],Vs[11][3],Bas[11][3],Bgs[11][3];
  double dpContainer[10][3],dqContainer[10][4],dvContainer[10][3];
  double JContainer[10][15][15],PContainer[10][15][15],baContainer[10][3],bgContainer[10][3],dtContainer[10];
  double icp2last[WINDOW_SIZE][7], icp2base[7],g_sum[WINDOW_SIZE+1][3];
  double ICP_N_t ,ICP_N_q;

  double* inMatrix0,*inMatrix1,*inMatrix2,*inMatrix3,*inMatrix4,*inMatrix5,*inMatrix6;
  double* inMatrix7,*inMatrix8,*inMatrix9,*inMatrix10,*inMatrix11,*inMatrix12,*inMatrix13;
  double* inMatrix14,*inMatrix15, *inMatrix17, *inMatrix18, *inMatrix19;
  
  inMatrix0 = mxGetPr(prhs[0]);
  inMatrix1 = mxGetPr(prhs[1]);
  inMatrix2 = mxGetPr(prhs[2]);
  inMatrix3 = mxGetPr(prhs[3]);
  inMatrix4 = mxGetPr(prhs[4]);
  inMatrix5 = mxGetPr(prhs[5]);
  inMatrix6 = mxGetPr(prhs[6]);
  inMatrix7 = mxGetPr(prhs[7]);
  inMatrix8 = mxGetPr(prhs[8]);
  inMatrix9 = mxGetPr(prhs[9]);
  inMatrix10 = mxGetPr(prhs[10]);
  inMatrix11 = mxGetPr(prhs[11]);
  inMatrix12 = mxGetPr(prhs[12]);
  inMatrix13 = mxGetPr(prhs[13]);
  inMatrix14 = mxGetPr(prhs[14]);
  inMatrix15 = mxGetPr(prhs[15]);
  inMatrix17 = mxGetPr(prhs[17]);
  inMatrix18 = mxGetPr(prhs[18]);
  inMatrix19 = mxGetPr(prhs[19]);
          
  memcpy(Ps, inMatrix0, sizeof(Ps));
  memcpy(Rs, inMatrix1, sizeof(Rs));
  memcpy(Qs, inMatrix2, sizeof(Qs));
  memcpy(Vs, inMatrix3, sizeof(Vs));
  memcpy(Bas, inMatrix4, sizeof(Bas));
  memcpy(Bgs, inMatrix5, sizeof(Bgs));
  memcpy(dpContainer, inMatrix6, sizeof(dpContainer));
  memcpy(dqContainer, inMatrix7, sizeof(dqContainer));
  memcpy(dvContainer, inMatrix8, sizeof(dvContainer));
  memcpy(JContainer, inMatrix9, sizeof(JContainer));
  memcpy(PContainer, inMatrix10, sizeof(PContainer));
  memcpy(baContainer, inMatrix11, sizeof(baContainer));
  memcpy(bgContainer, inMatrix12, sizeof(bgContainer));
  memcpy(dtContainer, inMatrix13, sizeof(dtContainer));
  memcpy(icp2last, inMatrix14, sizeof(icp2last));
  memcpy(icp2base, inMatrix15, sizeof(icp2base));
  memcpy(g_sum, inMatrix17, sizeof(g_sum));
  ICP_N_t = *inMatrix18;
  ICP_N_q = *inMatrix19;
  
  for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i][0];
        para_Pose[i][1] = Ps[i][1];
        para_Pose[i][2] = Ps[i][2];
        
        para_Pose[i][3] = Qs[i][1];
        para_Pose[i][4] = Qs[i][2];
        para_Pose[i][5] = Qs[i][3];
        para_Pose[i][6] = Qs[i][0];

        para_SpeedBias[i][0] = Vs[i][0];
        para_SpeedBias[i][1] = Vs[i][1];
        para_SpeedBias[i][2] = Vs[i][2];

        para_SpeedBias[i][3] = Bas[i][0];
        para_SpeedBias[i][4] = Bas[i][1];
        para_SpeedBias[i][5] = Bas[i][2];

        para_SpeedBias[i][6] = Bgs[i][0];
        para_SpeedBias[i][7] = Bgs[i][1];
        para_SpeedBias[i][8] = Bgs[i][2];
    }
  
//   for (int i = 0; i <= WINDOW_SIZE; i++)
//     {
//         for (int j = 3; j <= 6; j++){
//             cout<<para_Pose[i][j]<<endl;
//         }
//     }
  
  ceres::Problem problem;
//   ceres::LossFunction *loss_function;
//   //loss_function = new ceres::HuberLoss(1.0);
//   loss_function = new ceres::CauchyLoss(1.0);
  
  // add paramterblocks
  for (int i = 0; i < frame_count; i++)
  {// localparameterization for quanternion https://groups.google.com/g/ceres-solver/c/7HfF6DnCv7o
      ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
      problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
      problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
  }
  
  // save parameters into eigen matrix/vector
  Eigen::Vector3d dp[WINDOW_SIZE];
//   mwSize ninput_dimension0 = mxGetNumberOfDimensions(prhs[6]);
//   const mwSize * input_dimension0 = mxGetDimensions(prhs[6]);
//   cout<<"number of dimensions of 7th input paramters:"<< ninput_dimension0<<"\n";
//   for(mwSize i=0;i<ninput_dimension0;i++){
//         cout<<input_dimension0[i]<<"\n";}
//   cout<<sizeof(dpContainer)/sizeof(dpContainer[0])<<endl<<sizeof(dpContainer[0])/sizeof(dpContainer[0][0])<<"\n";
//   cout<<dpContainer[0][1]<<endl;
          
  for (int i = 0; i < WINDOW_SIZE; i++)
  {
      dp[i]<<dpContainer[i][0],dpContainer[i][1],dpContainer[i][2];
      //dp[0]<<1,2,3;
  }
  //cout<<dp[0]<<endl<<dp[1]<<endl<<dp[5]<<endl<<dp[7]<<endl<<dp[9]<<endl;
  
  Eigen::Quaterniond dq[WINDOW_SIZE];
  for (int i = 0; i < WINDOW_SIZE; i++)
  {
      dq[i]= Eigen::Quaterniond(dqContainer[i][0],dqContainer[i][1],dqContainer[i][2],dqContainer[i][3]);
      //dp[0]<<1,2,3;
  }
//   for (int i = 0; i < WINDOW_SIZE; i++)
//   {
//       cout<<dq[i].w()<<endl<<dq[i].vec()<<endl<<" "<<endl;
//   }
  
  Eigen::Vector3d dv[WINDOW_SIZE];
          
  for (int i = 0; i < WINDOW_SIZE; i++)
  {
      dv[i]<<dvContainer[i][0],dvContainer[i][1],dvContainer[i][2];
      //dp[0]<<1,2,3;
  }
//   for (int i = 0; i < WINDOW_SIZE; i++)
//   {
//       cout<<dv[i]<<endl<<" "<<endl;
//   }
  
  Eigen::Vector3d ba[WINDOW_SIZE];
  Eigen::Vector3d bg[WINDOW_SIZE];    
  
  for (int i = 0; i < WINDOW_SIZE; i++)
  {
      ba[i]<<baContainer[i][0],baContainer[i][1],baContainer[i][2];
      //dp[0]<<1,2,3;
      bg[i]<<bgContainer[i][0],bgContainer[i][1],bgContainer[i][2];
  }
  
//   for (int i = 0; i < WINDOW_SIZE; i++)
//   {
//       cout<<"ba:"<<ba[i]<<endl<<" "<<endl;
//       cout<<"bg:"<<bg[i]<<endl<<" "<<endl;
//   }
      
  Eigen::Matrix<double, 15, 15> jacobian[WINDOW_SIZE];
  Eigen::Matrix<double, 15, 15> covariance[WINDOW_SIZE];
  for (int i = 0; i < WINDOW_SIZE; i++)
  {
      for (int j = 0; j < 15; j++)
      {
        for (int k = 0; k < 15; k++)
        {
                  jacobian[i](j,k)=JContainer[i][k][j];
                  covariance[i](j,k)=PContainer[i][k][j];
                  //jacobian[i]<<k;  
//                   cout<<JContainer[i][k][j]<<" ";
//                   if(k==14){cout<<endl;}
        }
      }
  }
  //cout<<endl<<jacobian[2]<<endl;
//   cout<<endl<<covariance[0]<<endl;
//   cout<<endl<<covariance[4]<<endl;
//   cout<<endl<<covariance[9]<<endl;

//   for (int i = 0; i < WINDOW_SIZE; i++)
//   {
//       cout<<jacobian[i]<<endl<<" "<<endl;
//   }

//   for (int i = 0; i < WINDOW_SIZE; i++)
//   {
//       cout<<dtContainer[i]<<endl<<" "<<endl;
//   }
  
  Eigen::Vector3d dpICP2last[WINDOW_SIZE];
  Eigen::Quaterniond dqICP2last[WINDOW_SIZE];
  Eigen::Vector3d dpICP2base;
  Eigen::Quaterniond dqICP2base;
  for (int i = 0; i < WINDOW_SIZE; i++)
  {
      dpICP2last[i]<<icp2last[i][0],icp2last[i][1],icp2last[i][2];
      dqICP2last[i]= Eigen::Quaterniond(icp2last[i][6],icp2last[i][3],icp2last[i][4],icp2last[i][5]);
  }
  dpICP2base<<icp2base[0],icp2base[1],icp2base[2];
  dqICP2base = Eigen::Quaterniond(icp2base[6],icp2base[3],icp2base[4],icp2base[5]);
  
//   for (int i = 0; i < frame_count-1; i++)
//   {
//       cout<<dpICP2last[i]<<" "<<endl;
//       cout<<dqICP2last[i].w()<<endl<<dqICP2last[i].vec()<<endl<<" "<<endl;
//   }
//   cout<<dpICP2base<<" "<<endl;
//   cout<<dqICP2base.w()<<endl<<dqICP2base.vec()<<endl<<" "<<endl;
  
    
  Eigen::Vector3d deltaG_sum[WINDOW_SIZE+1];

  for (int i = 0; i <= WINDOW_SIZE; i++)
  {
      deltaG_sum[i]<<g_sum[i][0],g_sum[i][1],g_sum[i][2];
  }
  
//   for (int i = 0; i < frame_count; i++)
//   {
//       cout<<deltaG_sum[i]<<" "<<endl;
//   }


  
# if 1  
  // add residualblocks
//   for (int i = 0; i < frame_count-1; i++)
//     {
//         int j = i + 1;
// //         if (pre_integrations[j]->sum_dt > 10.0)
// //             continue;
//         // vins里面imufactor一共十一个取后十个，我们这里matlab直接给了后十个
// //         cout<<dp[i]<<endl<<dq[i].w()<<endl<<dq[i].vec()<<endl<<dv[i]<<endl<<ba[i]<<endl<<bg[i]<<endl;
// //         cout<<jacobian[i]<<endl<<covariance[i]<<endl<<dtContainer[i]<<endl<<deltaG_sum[j]<<endl;
// //         cout<<para_Pose[i]<<endl<<para_SpeedBias[i]<<endl<<para_Pose[j]<<endl<<para_SpeedBias[j]<<endl;
//         IMUFactor* imu_factor = new IMUFactor(dp[i],dq[i],dv[i],ba[i],bg[i],jacobian[i],covariance[i],dtContainer[i],deltaG_sum[j]);
//         problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
//     }
        
  for (int i = 0; i < frame_count-1; i++)
    {
        int j = i + 1;
//         if (pre_integrations[j]->sum_dt > 10.0)
//             continue;
        // vins里面imufactor一共十一个取后十个，我们这里matlab直接给了后十个
        ceres::CostFunction *cost_function = PCfactor::Create(dpICP2last[i], dqICP2last[i],ICP_N_t,ICP_N_q);
        problem.AddResidualBlock(cost_function, NULL, para_Pose[i], para_Pose[j]);
    }
//   ceres::CostFunction *cost_function_base = PCfactor::Create(dpICP2base, dqICP2base,ICP_N_t,ICP_N_q);
//   problem.AddResidualBlock(cost_function_base, NULL, para_Pose[0], para_Pose[frame_count-1]);
  
        
  ceres::Solver::Options options;
  
  options.linear_solver_type = ceres::DENSE_SCHUR;
  //options.num_threads = 2;
  //options.trust_region_strategy_type = ceres::DOGLEG;
  options.max_num_iterations = NUM_ITERATIONS;
  //options.use_explicit_schur_complement = true;
  options.minimizer_progress_to_stdout = true;
  //options.use_nonmonotonic_steps = true;
  // JF edited
//   options.gradient_tolerance = 1e-25;
//   options.parameter_tolerance = 1e-20;
  
  if (true)//marginalization_flag == MARGIN_OLD
      options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
  else
      options.max_solver_time_in_seconds = SOLVER_TIME;

  ceres::Solver::Summary summary;
  chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  ceres::Solve(options, &problem, &summary);
  chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  cout << "solve time cost = " << time_used.count() << " seconds. " << endl;
  
  // 输出结果
//   cout << summary.BriefReport() << endl;
  cout << summary.FullReport() << endl;
  
  plhs[0] = mxCreateDoubleMatrix(WINDOW_SIZE+1,SIZE_POSE, mxREAL);
  double *_ptr0 = (double*)mxGetPr(plhs[0]); // 获取矩阵数据指针
  memcpy(_ptr0, para_Pose, sizeof(para_Pose));
  plhs[1] = mxCreateDoubleMatrix(WINDOW_SIZE+1,SIZE_SPEEDBIAS, mxREAL);
  double *_ptr1 = (double*)mxGetPr(plhs[1]); // 获取矩阵数据指针
  memcpy(_ptr1, para_SpeedBias, sizeof(para_SpeedBias));
# endif
//   for(mwSize i=0;i<WINDOW_SIZE + 1;i++){
//       for (mwSize j=0;j<SIZE_POSE;j++){
//           cout<<para_Pose[i][j]<<" ";
//           if(j==SIZE_POSE-1){
//               cout<<"\n";
//           }
//       }
//   }
//   for(mwSize i=0;i<WINDOW_SIZE + 1;i++){
//       for (mwSize j=0;j<SIZE_SPEEDBIAS;j++){
//           cout<<para_SpeedBias[i][j]<<" ";
//           if(j==SIZE_SPEEDBIAS-1){
//               cout<<"\n";
//           }
//       }
//   }
  
# endif  
  
// #if 0 
//   mxDouble* vin1 = mxGetPr(prhs[0]);
//   mxDouble* vin2 = mxGetPr(prhs[1]);
//   mxDouble* vin3 = mxGetPr(prhs[2]);
//   
//   double ar = * vin1, br = * vin2, cr = * vin3;         // 真实参数值
//   double ae = 2.0, be = -1.0, ce = 5.0;        // 估计参数值
//   int N = 100;                                 // 数据点
//   double w_sigma = 1.0;                        // 噪声Sigma值
//   double inv_sigma = 1.0 / w_sigma;
//   //cv::RNG rng;                                 // OpenCV随机数产生器
// 
//   vector<double> x_data, y_data;      // 数据
//   for (int i = 0; i < N; i++) {
//     double x = i / 100.0;
//     x_data.push_back(x);
//     y_data.push_back(exp(ar * x * x + br * x + cr));
//     //y_data.push_back(exp(ar * x * x + br * x + cr) + rng.gaussian(w_sigma * w_sigma));
//   }
// 
//   double abc[3] = {ae, be, ce};
// 
//   // 构建最小二乘问题
//   ceres::Problem problem;
//   for (int i = 0; i < N; i++) {
//     problem.AddResidualBlock(     // 向问题中添加误差项
//       // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
//       new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(
//         new CURVE_FITTING_COST(x_data[i], y_data[i])
//       ),
//       nullptr,            // 核函数，这里不使用，为空
//       abc                 // 待估计参数
//     );
//   }
// 
//   // 配置求解器
//   ceres::Solver::Options options;     // 这里有很多配置项可以填
//   options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;  // 增量方程如何求解
//   options.minimizer_progress_to_stdout = true;   // 输出到cout
// 
//   ceres::Solver::Summary summary;                // 优化信息
//   chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
//   ceres::Solve(options, &problem, &summary);  // 开始优化
//   chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
//   chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
//   cout << "solve time cost = " << time_used.count() << " seconds. " << endl;
// 
//   // 输出结果
// //   cout << summary.BriefReport() << endl;
// //   cout << "estimated a,b,c = ";
// //   for (auto a:abc) cout << a << " ";
// //   cout << endl;
//    plhs[0] = mxCreateDoubleMatrix(1, 3, mxREAL);
//    double *_ptr = (double*)mxGetPr(plhs[0]); // 获取矩阵数据指针
//    memcpy(_ptr, abc, sizeof(double)*3);
// #endif  
//  return 0;
}