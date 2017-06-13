//Software License Agreement (BSD License)

//Copyright (c) 2008, Willow Garage, Inc.
//All rights reserved.

//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:

// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of Willow Garage, Inc. nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.

//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.

// Adaptation to gbM by Daniel Sidobre
//Copyright (c) 2010 LAAS-CNRS


//#include <angles/angles.h>
//#include <pr2_arm_kinematics/pr2_arm_ik.h>

#include "math.h"
#include <stdio.h>
#include "gb.h"

/**** List of angles (for reference) *******
      th1 = shoulder/turret pan
      th2 = shoulder/turret lift/pitch
      th3 = shoulder/turret roll
      th4 = elbow pitch
      th5 = elbow roll 
      th6 = wrist pitch
      th7 = wrist roll 
*****/
// 
// bool PR2ArmIK::init(const urdf::Model &robot_model, const std::string &root_name, const std::string &tip_name)
// {
//   std::vector<urdf::Pose> link_offset;
//   int num_joints = 0;
//   boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_name);
//   while(link && num_joints < 7)
//   {
//     boost::shared_ptr<const urdf::Joint> joint = robot_model.getJoint(link->parent_joint->name);
//     if(!joint)
//     {
//       ROS_ERROR("Could not find joint: %s",link->parent_joint->name.c_str());
//       return false;
//     }
//     if(joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
//     {
//       link_offset.push_back(link->parent_joint->parent_to_joint_origin_transform);
//       angle_multipliers_.push_back(joint->axis.x*fabs(joint->axis.x) +  joint->axis.y*fabs(joint->axis.y) +  joint->axis.z*fabs(joint->axis.z));
//       ROS_DEBUG("Joint axis: %d, %f, %f, %f",6-num_joints,joint->axis.x,joint->axis.y,joint->axis.z);
//       if(joint->type != urdf::Joint::CONTINUOUS)
//       {
//         min_angles_.push_back(joint->safety->soft_lower_limit);
//         max_angles_.push_back(joint->safety->soft_upper_limit);
//         continuous_joint_.push_back(false);
//       }
//       else
//       {
//         min_angles_.push_back(-M_PI);
//         max_angles_.push_back(M_PI);
//         continuous_joint_.push_back(true);
//       }
//       addJointToChainInfo(link->parent_joint,solver_info_);
//       num_joints++;
//     }
//     link = robot_model.getLink(link->getParent()->name);
//   } 
// 
//   solver_info_.link_names.push_back(tip_name);
// 
//   //  solver_info_.link_names.push_back(tip_name);
//   // We expect order from root to tip, so reverse the order
//   std::reverse(angle_multipliers_.begin(),angle_multipliers_.end());
//   std::reverse(min_angles_.begin(),min_angles_.end());
//   std::reverse(max_angles_.begin(),max_angles_.end());
//   std::reverse(link_offset.begin(),link_offset.end());
//   std::reverse(solver_info_.limits.begin(),solver_info_.limits.end());
//   std::reverse(solver_info_.joint_names.begin(),solver_info_.joint_names.end());
//   std::reverse(solver_info_.link_names.begin(),solver_info_.link_names.end());
//   std::reverse(continuous_joint_.begin(),continuous_joint_.end());
// 
//   if(num_joints != 7)
//   {
//     ROS_FATAL("PR2ArmIK:: Chain from %s to %s does not have 7 joints",root_name.c_str(),tip_name.c_str());
//     return false;
//   }
// 
// /*   torso_shoulder_offset_x_ = link_offset[0].position.x; */
// /*   torso_shoulder_offset_y_ = link_offset[0].position.y; */
// /*   torso_shoulder_offset_z_ = link_offset[0].position.z; */
//   // a1
//   shoulder_upperarm_offset_ = distance(link_offset[1]);
//   // r3
//   upperarm_elbow_offset_ = distance(link_offset[3]);
//   // r5
//   elbow_wrist_offset_ = distance(link_offset[5]);
//   // (a1 + r3)
//   shoulder_elbow_offset_ = shoulder_upperarm_offset_ + upperarm_elbow_offset_;
//   // (a1 + r3 + r5)
//   shoulder_wrist_offset_ = shoulder_upperarm_offset_+upperarm_elbow_offset_+elbow_wrist_offset_;
// 
//   Eigen::Matrix4f home = Eigen::Matrix4f::Identity();
//   home(0,3) = shoulder_upperarm_offset_ +  upperarm_elbow_offset_ +  elbow_wrist_offset_;
//   home_inv_ = home.inverse();
//   grhs_ = home;
//   gf_ = home_inv_;
//   solution_.resize( NUM_JOINTS_ARM7DOF);
//   return true;
// }

// void PR2ArmIK::addJointToChainInfo(boost::shared_ptr<const urdf::Joint> joint, kinematics_msgs::KinematicSolverInfo &info)
// {
//   motion_planning_msgs::JointLimits limit;
//   info.joint_names.push_back(joint->name);//Joints are coming in reverse order
//   limit.min_position = joint->safety->soft_lower_limit;
//   limit.max_position = joint->safety->soft_upper_limit;
//   if(joint->type != urdf::Joint::CONTINUOUS)
//   {
//     limit.min_position = joint->safety->soft_lower_limit;
//     limit.max_position = joint->safety->soft_upper_limit;
//     limit.has_position_limits = true;
//   }
//   else
//   {
//     limit.min_position = -M_PI;
//     limit.max_position = M_PI;
//     limit.has_position_limits = false;
//   }
//   limit.max_velocity = joint->limits->velocity;
//   limit.has_velocity_limits = 1;
//   info.limits.push_back(limit);
// }

// void PR2ArmIK::getSolverInfo(kinematics_msgs::KinematicSolverInfo &info)
// {
//   info = solver_info_;
// }


// void PR2ArmIK::computeIKShoulderPan(const Eigen::Matrix4f &g_in, const double &t1_in)
// {
// //t1 = shoulder/turret pan is specified
//   solution_ik_.clear();
// 
//   Eigen::Matrix4f g = g_in;
// //First bring everything into the arm frame
//   g(0,3) = g_in(0,3);// - torso_shoulder_offset_x_;
//   g(1,3) = g_in(1,3);// - torso_shoulder_offset_y_;
//   g(2,3) = g_in(2,3);// - torso_shoulder_offset_z_;
// 
//   double t1 = angles::normalize_angle(t1_in);
//   if(!checkJointLimits(t1,0))
//     return;
// 
// 
//   double cost1, cost2, cost3, cost4;
//   double sint1, sint2, sint3, sint4;
// 
//   gf_ = g*home_inv_;
// 
//   cost1 = cos(t1);
//   sint1 = sin(t1);
// 
//   double t2(0), t3(0), t4(0), t5(0), t6(0), t7(0);
// 
//   double at(0), bt(0), ct(0);
// 
//   double theta2[2],theta3[2],theta4[2],theta5[2],theta6[4],theta7[2]; 
// 
//   double sopx = a1*cost1;
//   double sopy = a1*sint1;
//   double sopz = 0;
// 
//   double x = g(0,3);
//   double y = g(1,3);
//   double z = g(2,3);
// 
//   double dx = x - sopx;
//   double dy = y - sopy;
//   double dz = z - sopz;
// 
//   double dd = dx*dx + dy*dy + dz*dz;
// 
//   double numerator = dd-a1*a1+2*a1*(a1+r3)-2*(a1+r3)*(a1+r3)+2*(a1+r3)*(a1+r3+r5)-(a1+r3+r5)*(a1+r3+r5);
//   double denominator = 2*(a1-(a1+r3))*((a1+r3)-(a1+r3+r5));
// 
//   double acosTerm = numerator/denominator;
// 
//   if (acosTerm > 1.0 || acosTerm < -1.0)
//     return;
// 
//   double acos_angle = acos(acosTerm);
//  
//   theta4[0] = acos_angle;
//   theta4[1] = -acos_angle;
// 
// #ifdef DEBUG
//   std::cout << "ComputeIK::theta3:" << numerator << "," << denominator << "," << std::endl << theta4[0] << std::endl;
// #endif
// 
//   for(int jj =0; jj < 2; jj++)
//   {
//     t4 = theta4[jj];
//     cost4 = cos(t4);
//     sint4 = sin(t4);
// 
// #ifdef DEBUG
//     std::cout << "t4 " << t4 << std::endl;
// #endif
//     if(std::isnan(t4))
//       continue;
// 
//     if(!checkJointLimits(t4,3))
//       continue;
// 
//     at = x*cost1+y*sint1-a1;
//     bt = -z;
//     ct = -a1 + (a1+r3) + ((a1+r3+r5)-(a1+r3))*cos(t4);
// 
//     if(!solveCosineEqn(at,bt,ct,theta2[0],theta2[1]))
//       continue;
// 
//     for(int ii=0; ii < 2; ii++)
//     {
//       t2 = theta2[ii];
//       if(!checkJointLimits(t2,1))
//         continue;
// 
// 
// #ifdef DEBUG
//       std::cout << "t2 " << t2 << std::endl; 
// #endif
//       sint2 = sin(t2);
//       cost2 = cos(t2);
// 
//       at = sint1*((a1+r3) - (a1+r3+r5))*sint2*sint4;
//       bt = (-(a1+r3)+(a1+r3+r5))*cost1*sint4;
//       ct = y - (a1+cost2*(-a1+(a1+r3)+(-(a1+r3)+(a1+r3+r5))*cos(t4)))*sint1;
//       if(!solveCosineEqn(at,bt,ct,theta3[0],theta3[1]))
//         continue;
// 
//       for(int kk =0; kk < 2; kk++)
//       {           
//         t3 = theta3[kk];
// 
//         if(!checkJointLimits(angles::normalize_angle(t3),2))
//           continue;
// 
//         sint3 = sin(t3);
//         cost3 = cos(t3);
// #ifdef DEBUG
//         std::cout << "t3 " << t3 << std::endl; 
// #endif
//         if(fabs((a1-(a1+r3)+((a1+r3)-(a1+r3+r5))*cost4)*sint2+((a1+r3)-(a1+r3+r5))*cost2*cost3*sint4-z) > IK_EPS )
//           continue;
// 
//         if(fabs(((a1+r3)-(a1+r3+r5))*sint1*sint3*sint4+cost1*(a1+cost2*(-a1+(a1+r3)+(-(a1+r3)+(a1+r3+r5))*cost4)+((a1+r3)-(a1+r3+r5))*cost3*sint2*sint4) - x) > IK_EPS)
//           continue;
// 
//         grhs_(0,0) = cost4*(gf_(0,0)*cost1*cost2+gf_(1,0)*cost2*sint1-gf_(2,0)*sint2)-(gf_(2,0)*cost2*cost3 + cost3*(gf_(0,0)*cost1 + gf_(1,0)*sint1)*sint2 + (-(gf_(1,0)*cost1) + gf_(0,0)*sint1)*sint3)*sint4;
// 
//         grhs_(0,1) = cost4*(gf_(0,1)*cost1*cost2 + gf_(1,1)*cost2*sint1 - gf_(2,1)*sint2) - (gf_(2,1)*cost2*cost3 + cost3*(gf_(0,1)*cost1 + gf_(1,1)*sint1)*sint2 + (-(gf_(1,1)*cost1) + gf_(0,1)*sint1)*sint3)*sint4;
// 
//         grhs_(0,2) = cost4*(gf_(0,2)*cost1*cost2 + gf_(1,2)*cost2*sint1 - gf_(2,2)*sint2) - (gf_(2,2)*cost2*cost3 + cost3*(gf_(0,2)*cost1 + gf_(1,2)*sint1)*sint2 + (-(gf_(1,2)*cost1) + gf_(0,2)*sint1)*sint3)*sint4;
// 
//         grhs_(1,0) = cost3*(gf_(1,0)*cost1 - gf_(0,0)*sint1) + gf_(2,0)*cost2*sint3 + (gf_(0,0)*cost1 + gf_(1,0)*sint1)*sint2*sint3;
// 
//         grhs_(1,1) = cost3*(gf_(1,1)*cost1 - gf_(0,1)*sint1) + gf_(2,1)*cost2*sint3 + (gf_(0,1)*cost1 + gf_(1,1)*sint1)*sint2*sint3;
// 
//         grhs_(1,2) = cost3*(gf_(1,2)*cost1 - gf_(0,2)*sint1) + gf_(2,2)*cost2*sint3 + (gf_(0,2)*cost1 + gf_(1,2)*sint1)*sint2*sint3;
// 
//         grhs_(2,0) = cost4*(gf_(2,0)*cost2*cost3 + cost3*(gf_(0,0)*cost1 + gf_(1,0)*sint1)*sint2 + (-(gf_(1,0)*cost1) + gf_(0,0)*sint1)*sint3) + (gf_(0,0)*cost1*cost2 + gf_(1,0)*cost2*sint1 - gf_(2,0)*sint2)*sint4;
// 
//         grhs_(2,1) = cost4*(gf_(2,1)*cost2*cost3 + cost3*(gf_(0,1)*cost1 + gf_(1,1)*sint1)*sint2 + (-(gf_(1,1)*cost1) + gf_(0,1)*sint1)*sint3) + (gf_(0,1)*cost1*cost2 + gf_(1,1)*cost2*sint1 - gf_(2,1)*sint2)*sint4;
// 
//         grhs_(2,2) = cost4*(gf_(2,2)*cost2*cost3 + cost3*(gf_(0,2)*cost1 + gf_(1,2)*sint1)*sint2 + (-(gf_(1,2)*cost1) + gf_(0,2)*sint1)*sint3) + (gf_(0,2)*cost1*cost2 + gf_(1,2)*cost2*sint1 - gf_(2,2)*sint2)*sint4;
// 
// 
//         double val1 = sqrt(grhs_(0,1)*grhs_(0,1)+grhs_(0,2)*grhs_(0,2));
//         double val2 = grhs_(0,0);
// 
//         theta6[0] = atan2(val1,val2);
//         theta6[1] = atan2(-val1,val2);
// 
// //            theta6[3] = M_PI + theta6[0];
// //            theta6[4] = M_PI + theta6[1];
// 
//         for(int mm = 0; mm < 2; mm++)
//         {
//           t6 = theta6[mm];
//           if(!checkJointLimits(angles::normalize_angle(t6),5))
//             continue;
// 
// #ifdef DEBUG
//           std::cout << "t6 " << t6 << std::endl;
// #endif
//           if(fabs(cos(t6) - grhs_(0,0)) > IK_EPS)
//             continue;
// 
//           if(fabs(sin(t6)) < IK_EPS)
//           {
//             //                std::cout << "Singularity" << std::endl;
//             theta5[0] = acos(grhs_(1,1))/2.0;
//             theta7[0] = theta7[0];
//             theta7[1] = M_PI+theta7[0];
//             theta5[1] = theta7[1];
//           }
//           else
//           {
//             theta7[0] = atan2(grhs_(0,1),grhs_(0,2));
//             theta5[0] = atan2(grhs_(1,0),-grhs_(2,0));
//             theta7[1] = M_PI+theta7[0];
//             theta5[1] = M_PI+theta5[0];
//           }
// #ifdef DEBUG
//           std::cout << "theta1: " << t1 << std::endl;
//           std::cout << "theta2: " << t2 << std::endl;
//           std::cout << "theta3: " << t3 << std::endl;
//           std::cout << "theta4: " << t4 << std::endl;
//           std::cout << "theta5: " << t5 << std::endl;
//           std::cout << "theta6: " << t6 << std::endl;
//           std::cout << "theta7: " << t7 << std::endl << std::endl << std::endl;
// #endif
//           for(int lll =0; lll < 2; lll++)
//           {
//             t5 = theta5[lll];
//             t7 = theta7[lll];
//             if(!checkJointLimits(t5,4))
//               continue;
//             if(!checkJointLimits(t7,6))
//               continue;
// 
// #ifdef DEBUG
//             std::cout << "t5" << t5 << std::endl;
//             std::cout << "t7" << t7 << std::endl;
// #endif      
//             if(fabs(sin(t6)*sin(t7)-grhs_(0,1)) > IK_EPS || fabs(cos(t7)*sin(t6)-grhs_(0,2)) > IK_EPS)
//               continue;
// 
//             solution_[0] = normalize_angle(t1)*angle_multipliers_[0];
//             solution_[1] = normalize_angle(t2)*angle_multipliers_[1];
//             solution_[2] = normalize_angle(t3)*angle_multipliers_[2];
//             solution_[3] = normalize_angle(t4)*angle_multipliers_[3];
//             solution_[4] = normalize_angle(t5)*angle_multipliers_[4];
//             solution_[5] = normalize_angle(t6)*angle_multipliers_[5];
//             solution_[6] = normalize_angle(t7)*angle_multipliers_[6];
//             solution_ik_.push_back(solution_);
// 
// #ifdef DEBUG
//             std::cout << "SOLN " << solution_[0] << " " << solution_[1] << " " <<  solution_[2] << " " << solution_[3] <<  " " << solution_[4] << " " << solution_[5] <<  " " << solution_[6] << std::endl << std::endl;
// #endif
//           }
//         }
//       }
//     }
//   }
// }

static double IK_EPS = 1e-5;

//   gbmPr2_direct computes geometry and differential direct models for PR2
// T01 = ( C1   -S1   0   0 )
//       ( S1    C1   0   0 )
//       (  0     0   1   0 )
// 
// T12 = ( C2   -S2   0  a1 )
//       (  0     0   1   0 )
//       (-S2   -C2   0   0 )
// 
// T23 = ( C3   -S3   0   0 )
//       (  0     0   1  r3 )
//       (-S3   -C3   0   0 )
// 
// T34 = ( C4   -S4   0   0 )
//       (  0     0  -1   0 )
//       ( S4    C4   0   0 )
// 
// T45 = ( C5   -S5   0   0 )
//       (  0     0   1  r5 )
//       (-S5   -C5   0   0 )
// 
// T56 = ( C6   -S6   0   0 )
//       (  0     0  -1   0 )
//       ( S6    C6   0   0 )
// 
// T67 = ( C7   -S7   0   0 )
//       (  0     0   1   0 )
//       ( -S7  -C7   0   0 )
// 
//                      T12 = ( C2   -S2   0  a1 )   f1= C1*C2
//                            (  0     0   1   0 )   f2= S1*C2
//                            (-S2   -C2   0   0 )   f3=-C1*S2
//  T01 = ( C1   -S1   0   0 )( f1    f3 -S1  f5 )   f4=-S1*S2
//        ( S1    C1   0   0 )( f2    f4  C1  f6 )   f5= C1*a1
//        (  0     0   1   0 )(-S2   -C2   0   0 )   f6= S1*a1
//  
//                      T23 = ( C3   -S3   0   0 )   f7=  f1*C3+S1*S3
//                            (  0     0   1  r3 )   f8=  f2*C3-C1*S3
//                            (-S3   -C3   0   0 )   f9= -S2*C3
//  T02 = ( f1    f3 -S1  f5 )( f7   f10  f3 f13 )   f10=-f1*S3+S1*C3
//        ( f2    f4  C1  f6 )( f8   f11  f4 f14 )   f11=-f2*S3-C1*C3
//        (-S2   -C2   0   0 )( f9   f12 -C2 f15 )   f12= S2*S3
//                                                   f13= f3*r3+f5
//                                                   f14= f4*r3+f6
//                                                   f15=-C2*r3
//
//                      T34 = ( C4   -S4   0   0 )   f16= f7*C4+f3*S4
//                            (  0     0  -1   0 )   f17= f8*C4+f4*S4
//                            ( S4    C4   0   0 )   f18= f9*C4-C2*S4
//  T03 = ( f7   f10  f3 f13 )( f16  f19 f10 f13 )   f19=-f7*S4+f3*C4
//        ( f8   f11  f4 f14 )( f17  f20 f11 f14 )   f20=-f8*S4+f4*C4
//        ( f9   f12 -C2 f15 )( f18  f21 f12 f15 )   f21=-f9*S4-C2*C4
//
//                      T45 = ( C5   -S5   0   0 )   f22= f16*C5-f10*S5
//                            (  0     0   1  r5 )   f23= f17*C5-f11*S5
//                            (-S5   -C5   0   0 )   f24= f18*C5-f12*S5
//  T04 = ( f16  f19 f10 f13 )( f22  f25 f19 f28 )   f25=-f16*S5-f10*C5
//        ( f17  f20 f11 f14 )( f23  f26 f20 f29 )   f26=-f17*S5-f11*C5
//        ( f18  f21 f12 f15 )( f24  f27 f21 f30 )   f27=-f18*S5-f12*C5
//                                                   f28= f19*r5+f13
//                                                   f29= f20*r5+f14
//                                                   f30= f21*r5+f15   
//  
//                      T56 = ( C6   -S6   0   0 )   f31= f22*C6+f19*S6
//                            (  0     0  -1   0 )   f32= f23*C6+f20*S6
//                            ( S6    C6   0   0 )   f33= f24*C6+f21*S6
//  T05 = ( f22  f25 f19 f28 )( f31  f34-f25 f28 )   f34=-f22*S6+f19*C6
//        ( f23  f26 f20 f29 )( f32  f35-f26 f29 )   f35=-f23*S6+f20*C6
//        ( f24  f27 f21 f30 )( f33  f36-f27 f30 )   f36=-f24*S6+f21*C6
//  
//                      T67 = ( C7   -S7   0   0 )   f37= f31*C7+f25*S7
//                            (  0     0   1   0 )   f38= f32*C7+f26*S7
//                            ( -S7  -C7   0   0 )   f39= f33*C7+f27*S7
//  T06 = ( f31  f34-f25 f28 )( f37  f40 f34 f28 )   f40=-f31*S7+f25*C7
//        ( f32  f35-f26 f29 )( f38  f41 f35 f29 )   f41=-f32*S7+f26*C7
//        ( f33  f36-f27 f30 )( f39  f42 f36 f30 )   f42=-f33*S7+f27*C7
//  
//  Po= O_0O_7 (position of the wrist center relatively to frame R_0) 
//  P2= O_20_7 (position of the wrist center relatively to frame R_2) = r3 * z3 + r5 * z5 
//  P3= 0_3O_7 (position of the wrist center relatively to frame R_3) = r5 * z5
//  
//  Jac=( z1^Po  z2^P2  z3^P3  z4^P3    0    0   0  )
//      (   z1     z2     z3     z4    z5   z6  z7  )
//  
//  z1^Po = | 0   |f28   |-f29
//          | 0 ^ |f29 = | f28
//          | 1   |f30   | 0
//  P2 = | f43                P2= P1-a1*x1           f43= f28-a1*C1
//       | f44                                       f44= f29-a1*S1
//       | f30
//  z2^P2 = |-S1    |f43   | C1*f30
//          | C1  ^ |f44 = | S1*f30
//          | 0     |f30   |-S1*f44-C1*f43
//  
//  P3 = | f43-r3*f3  = | f45     P3= P2-r3*z3       f45= f43-r3*f3
//       | f44-r3*f4    | f46                        f46= f44-r3*f4
//       | f30+r3*C2    | f47                        f47= f30+r3*C2
//  z3^P3 = | f3    |f45   | f4*f47+C2*f46
//          | f4  ^ |f46 = |-C2*f45-f3*f47
//          |-C2    |f47   | f3*f46-f4*f45
//
//
//  z4^P3 = | f10   |f45 = | f11*f47-f12*f46
//          | f11 ^ |f46   | f12*f45-f10*f47
//          | f12   |f47   | f10*f46-f11*f45
//  


void gbmPr2_direct(Gb_q7* Q, double a1, double r3, double r5, Gb_th* th07, Gb_jac7 jac7)
{
  double C1 = cos (Q->q1);
  double C2 = cos (Q->q2);
  double C3 = cos (Q->q3);
  double C4 = cos (Q->q4);
  double C5 = cos (Q->q5);
  double C6 = cos (Q->q6);
  double C7 = cos (Q->q7);
  double S1 = sin (Q->q1);
  double S2 = sin (Q->q2);
  double S3 = sin (Q->q3);
  double S4 = sin (Q->q4);
  double S5 = sin (Q->q5);
  double S6 = sin (Q->q6);
  double S7 = sin (Q->q7);

  double f1= C1*C2;
  double f2= S1*C2;
  double f3=-C1*S2;
  double f4=-S1*S2;
  double f5= C1*a1;
  double f6= S1*a1;
  double f7=  f1*C3+S1*S3;
  double f8=  f2*C3-C1*S3;
  double f9= -S2*C3;
  double f10=-f1*S3+S1*C3;
  double f11=-f2*S3-C1*C3;
  double f12= S2*S3;
  double f13= f3*r3+f5;
  double f14= f4*r3+f6;
  double f15=-C2*r3;
  double f16= f7*C4+f3*S4;
  double f17= f8*C4+f4*S4;
  double f18= f9*C4-C2*S4;
  double f19=-f7*S4+f3*C4;
  double f20=-f8*S4+f4*C4;
  double f21=-f9*S4-C2*C4;
  double f22= f16*C5-f10*S5;
  double f23= f17*C5-f11*S5;
  double f24= f18*C5-f12*S5;
  double f25=-f16*S5-f10*C5;
  double f26=-f17*S5-f11*C5;
  double f27=-f18*S5-f12*C5;
  double f28= f19*r5+f13;
  double f29= f20*r5+f14;
  double f30= f21*r5+f15;
  double f31= f22*C6+f19*S6;
  double f32= f23*C6+f20*S6;
  double f33= f24*C6+f21*S6;
  double f34=-f22*S6+f19*C6;
  double f35=-f23*S6+f20*C6;
  double f36=-f24*S6+f21*C6;
  double f37= f31*C7+f25*S7;
  double f38= f32*C7+f26*S7;
  double f39= f33*C7+f27*S7;
  double f40=-f31*S7+f25*C7;
  double f41=-f32*S7+f26*C7;
  double f42=-f33*S7+f27*C7;
  double f43= f28-a1*C1;
  double f44= f29-a1*S1;
  double f45= f43-r3*f3;
  double f46= f44-r3*f4;
  double f47= f30+r3*C2;

  jac7.c1.x  = -f29;
  jac7.c1.y  =  f28;
  jac7.c1.z  =  0;
  jac7.c1.rx =  0;
  jac7.c1.ry =  0;
  jac7.c1.rz =  0;
  jac7.c2.x  =  C1*f30;
  jac7.c2.y  =  S1*f30;
  jac7.c2.z  = -S1*f44-C1*f43;
  jac7.c2.rx = -S1;
  jac7.c2.ry =  C1;
  jac7.c2.rz =  0;
  jac7.c3.x  =  f4*f47+C2*f46;
  jac7.c3.y  = -C2*f45-f3*f47;
  jac7.c3.z  =  f3*f46-f4*f45;
  jac7.c3.rx =  f3;
  jac7.c3.ry =  f4;
  jac7.c3.rz = -C2;
  jac7.c4.x  =  f11*f47-f12*f46;
  jac7.c4.y  =  f12*f45-f10*f47;
  jac7.c4.z  =  f10*f46-f11*f45;
  jac7.c4.rx =  f10;
  jac7.c4.ry = 	f11;
  jac7.c4.rz = 	f12;
  jac7.c5.x  =  0;
  jac7.c5.y  =  0;
  jac7.c5.z  =  0;
  jac7.c5.rx =  f19;
  jac7.c5.ry =  f20;
  jac7.c5.rz =  f21;
  jac7.c6.x  =  0;
  jac7.c6.y  =  0;
  jac7.c6.z  =  0;
  jac7.c6.rx = -f25;
  jac7.c6.ry = -f26;
  jac7.c6.rz = -f27;
  jac7.c7.x  =  0;
  jac7.c7.y  =  0;
  jac7.c7.z  =  0;
  jac7.c7.rx =  f34;
  jac7.c7.ry =  f35;
  jac7.c7.rz =  f36;

  th07->vx.x = f37;
  th07->vx.y = f38;
  th07->vx.z = f39;
  th07->vy.x = f40;
  th07->vy.y = f41;
  th07->vy.z = f42;
  th07->vz.x = f34;
  th07->vz.y = f35;
  th07->vz.z = f36;
  th07->vp.x = f28;
  th07->vp.y = f29;
  th07->vp.z = f30;
}


static inline double normalize_angle_positive(double angle)
{
  return fmod(fmod(angle, 2.0*M_PI) + 2.0*M_PI, 2.0*M_PI);
}

static inline double normalize_angle(double angle)
{
  double a = normalize_angle_positive(angle);
  if (a > M_PI)
    a -= 2.0 *M_PI;
  return a;
}


int gbmPr2CheckJointLimits(Gb_q7* qMin, Gb_q7* qMax, Gb_q7* q)
{
  return gbmPr2CheckJointLimit(qMin->q1, qMax->q1, q->q1)
    && gbmPr2CheckJointLimit(qMin->q2, qMax->q2, q->q2)
    && gbmPr2CheckJointLimit(qMin->q3, qMax->q3, q->q3)
    && gbmPr2CheckJointLimit(qMin->q4, qMax->q4, q->q4)
    && gbmPr2CheckJointLimit(qMin->q5, qMax->q5, q->q5)
    && gbmPr2CheckJointLimit(qMin->q6, qMax->q6, q->q6)
    && gbmPr2CheckJointLimit(qMin->q7, qMax->q7, q->q7);
}

int gbmPr2CheckJointLimit(double min, double max, double q)
{
  if ( (q >= min) && (q <= max) ) {
    return 1;
  } else {
    return 0;
  }
}
int gbmSolveQuadratic(double a, double b, double c, double *x1, double *x2)
{
  double discriminant = b*b-4*a*c;
  if(fabs(a) < IK_EPS) {
    *x1 = -c/b;
    *x2 = *x1;
    return 1;
  }
  //ROS_DEBUG("Discriminant: %f\n",discriminant);
  if (discriminant >= 0) {      
    *x1 = (-b + sqrt(discriminant))/(2*a); 
    *x2 = (-b - sqrt(discriminant))/(2*a);
    return 1;
  } else {
    if(fabs(discriminant) < IK_EPS) {
      *x1 = -b/(2*a);
      *x2 = -b/(2*a);
      return 1;
    } else {
      *x1 = -b/(2*a);
      *x2 = -b/(2*a);
      return 0;
    }
  }
}
int gbmSolveCosineEqn(double a, double b, double c, double* soln1, double* soln2)
{
  double theta1 = atan2(b,a);
  double denom  = sqrt(a*a+b*b);

  if(fabs(denom) < IK_EPS) {
    // should never happen, wouldn't make sense but make sure it is checked nonetheless
#ifdef DEBUG
    std::cout << "denom: " << denom << std::endl;
#endif
    return 0;
  }
  double rhs_ratio = c/denom;
  if(rhs_ratio < -1 || rhs_ratio > 1) {
#ifdef DEBUG
    std::cout << "rhs_ratio: " << rhs_ratio << std::endl;
#endif
    return 0;
  }
  double acos_term = acos(rhs_ratio);
  *soln1 = theta1 + acos_term;
  *soln2 = theta1 - acos_term;
  return 1;
}


// a1=0.1; r3=0.45;  r5=??
Gb_statusMGI pr2_mgi_q3_8(Gb_th* th07, Gb_q7* Qp,
			  double a1, double r3, double r5, Gb_q7 *qMin, Gb_q7 *qMax, 
			  double epsilon, Gb_q7 qsol[32], int* nbsolution) 
{
  //void PR2ArmIK::computeIKShoulderRoll(const Eigen::Matrix4f &g_in, const double &t3)
  //{
  //t3 = shoulder/turret roll is specified
  //Eigen::Matrix4f g = g_in;
//First bring everything into the arm frame

  double t3 = Qp->q3;
  if (gbmPr2CheckJointLimit(qMin->q3, qMax->q3, t3) == 0) {
    return MGI_ERROR;
  }
  double x = th07->vp.x;
  double y = th07->vp.y;
  double z = th07->vp.z;
  double cost1, cost2, cost3, cost4;
  double sint1, sint2, sint3, sint4;

  *nbsolution = 0;

  // home= 1 0 0 a1+r3+r5    home_inv_= 1 0 0 -a1-r3-r5
  //       0 1 0    0                   0 1 0     0
  //       0 0 1    0                   0 0 1     0
  // gf_ = g*home_inv_;
  Gb_th gf;
  Gb_th grhs;
  gf.vx.x = th07->vx.x;
  gf.vx.y = th07->vx.y;
  gf.vx.z = th07->vx.z;
  gf.vy.x = th07->vy.x;
  gf.vy.y = th07->vy.y;
  gf.vy.z = th07->vy.z;
  gf.vz.x = th07->vz.x;
  gf.vz.y = th07->vz.y;
  gf.vz.z = th07->vz.z;
  gf.vp.x = th07->vp.x - th07->vx.x * (a1+r3+r5);
  gf.vp.y = th07->vp.y - th07->vx.y * (a1+r3+r5);
  gf.vp.z = th07->vp.z - th07->vx.z * (a1+r3+r5);

  cost3 = cos(t3);
  sint3 = sin(t3);

  double t1, t2, t4, t5, t6, t7;
  double at, bt, ct;

  double theta1[2],theta2[2],theta4[4],theta5[2],theta6[4],theta7[2];

  double c0 = -sin(-t3)*r5;
  double c1 = -cos(-t3)*r5;

  double d0 = 4*a1*a1*(r3*r3+c1*c1-z*z);
  double d1 = 8*a1*a1*r3*r5;
  double d2 = 4*a1*a1*(r5*r5-c1*c1);

  double b0 = x*x+y*y+z*z-a1*a1-r3*r3-c0*c0-c1*c1;
  double b1 = -2*r3*r5;

  if(!gbmSolveQuadratic(b1*b1-d2,2*b0*b1-d1,b0*b0-d0,&theta4[0],&theta4[1])) {
#ifdef DEBUG
    printf("No solution to quadratic eqn\n");
#endif
    return MGI_ERROR;
  }
  theta4[0] = acos(theta4[0]);
  theta4[2] = acos(theta4[1]);
  theta4[1] = -theta4[0];
  theta4[3] = -theta4[2];

  int jj;
  for(jj = 0; jj < 4; jj++) {
    t4 = theta4[jj];
    if (gbmPr2CheckJointLimit(qMin->q4, qMax->q4, t4) == 0) {
      continue;
    }
    cost4 = cos(t4);
    sint4 = sin(t4);
#ifdef DEBUG
    std::cout << "t4 " << t4 << std::endl;
#endif
    if(isnan(t4))
      continue;
    at = cos(t3)*sin(t4)*((a1+r3)-(a1+r3+r5));
    bt = (a1-(a1+r3)+((a1+r3)-(a1+r3+r5))*cos(t4));
    ct = z;

    if(!gbmSolveCosineEqn(at,bt,ct, &(theta2[0]), &(theta2[1]))) {
      continue;
    }
    int ii;
    for(ii=0; ii < 2; ii++) {
      t2 = theta2[ii];
#ifdef DEBUG
      std::cout << "t2 " << t2 << std::endl;
#endif
      if (gbmPr2CheckJointLimit(qMin->q2, qMax->q2, t2) == 0) {
        continue;
      }

      sint2 = sin(t2);
      cost2 = cos(t2);

      at = -y;
      bt = x;
      ct = ((a1+r3)-(a1+r3+r5))*sin(t3)*sin(t4);
      if(!gbmSolveCosineEqn(at,bt,ct, &(theta1[0]), &(theta1[1]))) {
#ifdef DEBUG
        std::cout << "could not solve cosine equation for t1" << std::endl;
#endif
        continue;
      }
      int kk;
      for(kk =0; kk < 2; kk++) {           
        t1 = theta1[kk];
#ifdef DEBUG
        std::cout << "t1 " << t1 << std::endl;
#endif
	if (gbmPr2CheckJointLimit(qMin->q1, qMax->q1, t1) == 0) {
          continue;
        }
        sint1 = sin(t1);
        cost1 = cos(t1);
        if(fabs((a1-(a1+r3)+((a1+r3)-(a1+r3+r5))*cost4)*sint2+((a1+r3)-(a1+r3+r5))*cost2*cost3*sint4-z) > IK_EPS )
        {
#ifdef DEBUG
          printf("z value not matched %f\n",fabs((a1-(a1+r3)+((a1+r3)-(a1+r3+r5))*cost4)*sint2+((a1+r3)-(a1+r3+r5))*cost2*cost3*sint4-z));
#endif
          continue;
        }
        if(fabs(((a1+r3)-(a1+r3+r5))*sint1*sint3*sint4+cost1*(a1+cost2*(-a1+(a1+r3)+(-(a1+r3)+(a1+r3+r5))*cost4)+((a1+r3)-(a1+r3+r5))*cost3*sint2*sint4) - x) > IK_EPS)
        {
#ifdef DEBUG
          printf("x value not matched by %f\n",fabs(((a1+r3)-(a1+r3+r5))*sint1*sint3*sint4+cost1*(a1+cost2*(-a1+(a1+r3)+(-(a1+r3)+(a1+r3+r5))*cost4)+((a1+r3)-(a1+r3+r5))*cost3*sint2*sint4) - x));
#endif
          continue;
        }
        if(fabs(-((a1+r3)-(a1+r3+r5))*cost1*sint3*sint4+sint1*(a1+cost2*(-a1+(a1+r3)+(-(a1+r3)+(a1+r3+r5))*cost4)+((a1+r3)-(a1+r3+r5))*cost3*sint2*sint4) - y) > IK_EPS)
        {
#ifdef DEBUG
          printf("y value not matched\n");
#endif
          continue;
        }
        grhs.vx.x =
	  cost4*(gf.vx.x*cost1*cost2+gf.vx.y*cost2*sint1-gf.vx.z*sint2)
	  -(gf.vx.z*cost2*cost3
	    + cost3*(gf.vx.x*cost1 + gf.vx.y*sint1)*sint2
	    + (-(gf.vx.y*cost1) + gf.vx.x*sint1)*sint3)*sint4;

        grhs.vy.x = cost4*(gf.vy.x*cost1*cost2 + gf.vy.y*cost2*sint1 - gf.vy.z*sint2) - (gf.vy.z*cost2*cost3 + cost3*(gf.vy.x*cost1 + gf.vy.y*sint1)*sint2 + (-(gf.vy.y*cost1) + gf.vy.x*sint1)*sint3)*sint4;

        grhs.vz.x = cost4*(gf.vz.x*cost1*cost2 + gf.vz.y*cost2*sint1 - gf.vz.z*sint2) - (gf.vz.z*cost2*cost3 + cost3*(gf.vz.x*cost1 + gf.vz.y*sint1)*sint2 + (-(gf.vz.y*cost1) + gf.vz.x*sint1)*sint3)*sint4;

        grhs.vx.y = cost3*(gf.vx.y*cost1 - gf.vx.x*sint1) + gf.vx.z*cost2*sint3 + (gf.vx.x*cost1 + gf.vx.y*sint1)*sint2*sint3;

        grhs.vy.y = cost3*(gf.vy.y*cost1 - gf.vy.x*sint1) + gf.vy.z*cost2*sint3 + (gf.vy.x*cost1 + gf.vy.y*sint1)*sint2*sint3;

        grhs.vz.y = cost3*(gf.vz.y*cost1 - gf.vz.x*sint1) + gf.vz.z*cost2*sint3 + (gf.vz.x*cost1 + gf.vz.y*sint1)*sint2*sint3;

        grhs.vx.z = cost4*(gf.vx.z*cost2*cost3 + cost3*(gf.vx.x*cost1 + gf.vx.y*sint1)*sint2 + (-(gf.vx.y*cost1) + gf.vx.x*sint1)*sint3) + (gf.vx.x*cost1*cost2 + gf.vx.y*cost2*sint1 - gf.vx.z*sint2)*sint4;

        grhs.vy.z = cost4*(gf.vy.z*cost2*cost3 + cost3*(gf.vy.x*cost1 + gf.vy.y*sint1)*sint2 + (-(gf.vy.y*cost1) + gf.vy.x*sint1)*sint3) + (gf.vy.x*cost1*cost2 + gf.vy.y*cost2*sint1 - gf.vy.z*sint2)*sint4;

        grhs.vz.z = cost4*(gf.vz.z*cost2*cost3 + cost3*(gf.vz.x*cost1 + gf.vz.y*sint1)*sint2 + (-(gf.vz.y*cost1) + gf.vz.x*sint1)*sint3) + (gf.vz.x*cost1*cost2 + gf.vz.y*cost2*sint1 - gf.vz.z*sint2)*sint4;


        double val1 = sqrt(grhs.vy.x*grhs.vy.x+grhs.vz.x*grhs.vz.x);
        double val2 = grhs.vx.x;

        theta6[0] = atan2(val1,val2);
        theta6[1] = atan2(-val1,val2);
	int mm;
        for(mm = 0; mm < 2; mm++) {
          t6 = theta6[mm];
#ifdef DEBUG
          std::cout << "t6 " << t6 << std::endl;
#endif
	  if (gbmPr2CheckJointLimit(qMin->q6, qMax->q6, t6) == 0) {
            continue;
          }

          if(fabs(cos(t6) - grhs.vx.x) > IK_EPS)
            continue;

          if(fabs(sin(t6)) < IK_EPS) {
            //                std::cout << "Singularity" << std::endl;
            theta5[0] = acos(grhs.vy.y)/2.0;
            theta7[0] = theta5[0];
//            theta7[1] = M_PI+theta7[0];
//            theta5[1] = theta7[1];
          } else {
            theta7[0] = atan2(grhs.vy.x/sin(t6),grhs.vz.x/sin(t6));
            theta5[0] = atan2(grhs.vx.y/sin(t6),-grhs.vx.z/sin(t6));
//            theta7[1] = M_PI+theta7[0];
//            theta5[1] = M_PI+theta5[0];
          }
	  int lll;
          for(lll =0; lll < 1; lll++) {
            t5 = theta5[lll];
            t7 = theta7[lll];

	    if (gbmPr2CheckJointLimit(qMin->q5, qMax->q5, t5) == 0) {
              continue;
            }
	    if (gbmPr2CheckJointLimit(qMin->q7, qMax->q7, t7) == 0) {
              continue;
            }
#ifdef DEBUG
            std::cout << "t5 " << t5 << std::endl;
            std::cout << "t7 " << t7 << std::endl;
#endif      
            //           if(fabs(sin(t6)*sin(t7)-grhs.vy.x) > IK_EPS || fabs(cos(t7)*sin(t6)-grhs.vz.x) > IK_EPS)
            //  continue;

#ifdef DEBUG
            std::cout << "theta1: " << t1 << std::endl;
            std::cout << "theta2: " << t2 << std::endl;
            std::cout << "theta3: " << t3 << std::endl;
            std::cout << "theta4: " << t4 << std::endl;
            std::cout << "theta5: " << t5 << std::endl;
            std::cout << "theta6: " << t6 << std::endl;
            std::cout << "theta7: " << t7 << std::endl << std::endl << std::endl;
#endif
	    // ?? I suppose angle_multipliers_ is 1 !
            qsol[*nbsolution].q1 = normalize_angle(t1);
            qsol[*nbsolution].q2 = normalize_angle(t2);
            qsol[*nbsolution].q3 = t3;
            qsol[*nbsolution].q4 = normalize_angle(t4);
            qsol[*nbsolution].q5 = normalize_angle(t5);
            qsol[*nbsolution].q6 = normalize_angle(t6);
            qsol[*nbsolution].q7 = normalize_angle(t7);
	    (*nbsolution)++;
#ifdef DEBUG
            std::cout << "SOLN " << solution_[0] << " " << solution_[1] << " " <<  solution_[2] << " " << solution_[3] <<  " " << solution_[4] << " " << solution_[5] <<  " " << solution_[6] << std::endl << std::endl;
#endif
          }
        }
      }
    }
  }
  if (nbsolution > 0) {
    return MGI_OK;
  } else {
    return MGI_ERROR;
  }
}

