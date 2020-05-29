/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <cmath>
#include <iostream>

#include <towr/terrain/examples/height_map_examples.h>
#include <towr/nlp_formulation.h>
#include <ifopt/ipopt_solver.h>

#include <towr/initialization/gait_generator.h>
//#define base_height_initial 0.54
#define towr_initial_output false
using namespace towr;

// A minimal example how to build a trajectory optimization problem using TOWR.
//
// The more advanced example that includes ROS integration, GUI, rviz
// visualization and plotting can be found here:
// towr_ros/src/towr_ros_app.cc
struct Trajectory_data
{

float base_linear[3];
float base_angular[3];
float ee_linear[12]; // 4 legs 3 cordinates
float ee_force[12]; // 4 legs 3 componenst for each force


};





void towr_trajectory(NlpFormulation &formulation,SplineHolder &solution,
                    Eigen::Vector3d target,Eigen::Vector3d target_a,Eigen::Vector3d base_i_lin,
                    Eigen::Vector3d base_i_ang,Eigen::Vector3d ee1,
                    Eigen::Vector3d ee2,Eigen::Vector3d ee3,Eigen::Vector3d ee4,
                    Trajectory_data* data,int no_of_samples,int terrain_id,
                    int gait_pattern)
{




  // terrain
  //formulation.terrain_ = std::make_shared<FlatGround>(0.0);
  /*
  0                  FlatID,
  1                  BlockID,
  2                  StairsID,
  3                  GapID,
  4                  SlopeID,
  5                  ChimneyID,
  6                  ChimneyLRID,
  7                  TERRAIN_COUNT
  */
  auto Terrain_id = static_cast<HeightMap::TerrainID>(terrain_id);
  formulation.terrain_ = HeightMap::MakeTerrain(Terrain_id);
  // Kinematic limits and dynamic parameters of the Anymal
  formulation.model_ = RobotModel(RobotModel::Anymal);

  // set the initial position of the Anymal
  //Eigen::Vector3d Ini(-2,0,0.54);
  formulation.initial_base_.lin.at(kPos) = base_i_lin;//*(base_i_lin+0);
  formulation.initial_base_.ang.at(kPos) = base_i_ang;
  /*
  std::cout<<"\nBase_i_lin:\n"<<formulation.initial_base_.lin.at(kPos).x()
                          <<"\n"<<formulation.initial_base_.lin.at(kPos).y()
                          <<"\n"<<formulation.initial_base_.lin.at(kPos).z();
  std::cout<<"\nBase_i_ang:\n"<<formulation.initial_base_.ang.at(kPos).x()
                          <<"\n"<<formulation.initial_base_.ang.at(kPos).y()
                          <<"\n"<<formulation.initial_base_.ang.at(kPos).z();
*/
  auto nominal_stance_B = formulation.model_.kinematic_model_->GetNominalStanceInBase();
  double z_ground = 0.0;
  formulation.initial_ee_W_ =  nominal_stance_B;
 int ee_index = 0;
  std::for_each(formulation.initial_ee_W_.begin(), formulation.initial_ee_W_.end(),
                  [&](Eigen::Vector3d& p)
                  {switch(ee_index)
                  {
                    case(0):{p = ee1;break;}
                    case(1):{p = ee2;break;}
                    case(2):{p = ee3;break;}
                    case(3):{p = ee4;break;}
                    default:{break;}
                    
                    
                  } 
//std::cout<<"\nee"<<ee_index<<":\n"<<p;


                  ee_index ++; } // feet at 0 height
     );

 //formulation.initial_base_.lin.at(kPos).z() = - nominal_stance_B.front().z() + z_ground;
   int n_ee = formulation.model_.kinematic_model_->GetNumberOfEndeffectors();
   std::cout<<"\n\nn_ee :"<<n_ee<<"\n\n";
  // define the desired goal state of the Anymal
  formulation.final_base_.lin.at(towr::kPos) = target;
  formulation.final_base_.ang.at(towr::kPos) = target_a;
  

    auto gait_gen_ = GaitGenerator::MakeGaitGenerator(n_ee);
    
    // overlap-walk -0
    // fly trot - 1
    // pace - 2
    // bound - 3
    // gallop - 4
    GaitGenerator::Combos k = GaitGenerator::Combos(gait_pattern); //walk combo
    switch(k)
    {
     case(0):{std::cout<<"\n\nGait_selected:overlap-walk\n\n";break;}
     case(1):{std::cout<<"\n\nGait_selected:fly trot\n\n";break;}
     case(2):{std::cout<<"\n\nGait_selected:pace\n\n";break;}
     case(3):{std::cout<<"\n\nGait_selected:bound\n\n";break;}
     case(4):{std::cout<<"\n\nGait_selected:gallop\n\n";break;}
     default:break;
     }
    gait_gen_->SetCombo(k);
    double total_duration = 2.0;
    //random phase duration , actually monoped
    for (int ee=0; ee<n_ee; ++ee) 
    {
      formulation.params_.ee_phase_durations_.push_back(gait_gen_->GetPhaseDurations(total_duration, ee));
      formulation.params_.ee_in_contact_at_start_.push_back(gait_gen_->IsInContactAtStart(ee));
    }



  // Initialize the nonlinear-programming problem with the variables,
  // constraints and costs.
  ifopt::Problem nlp;

  for (auto c : formulation.GetVariableSets(solution))
    nlp.AddVariableSet(c);
  for (auto c : formulation.GetConstraints(solution))
    nlp.AddConstraintSet(c);
  for (auto c : formulation.GetCosts())
    nlp.AddCostSet(c);
 


  // You can add your own elements to the nlp as well, simply by calling:
  // nlp.AddVariablesSet(your_custom_variables);
  // nlp.AddConstraintSet(your_custom_constraints);

  // Choose ifopt solver (IPOPT or SNOPT), set some parameters and solve.
  // solver->SetOption("derivative_test", "first-order");
  auto solver = std::make_shared<ifopt::IpoptSolver>();
  solver->SetOption("jacobian_approximation", "exact"); // "finite difference-values"
  solver->SetOption("max_cpu_time", 40.0);
  solver->Solve(nlp);
 
  // Can directly view the optimization variables through:
  // Eigen::VectorXd x = nlp.GetVariableValues()
  // However, it's more convenient to access the splines constructed from these
  // variables and query their values at specific times:
  using namespace std;
  
  cout.precision(2);

  //nlp.PrintCurrent(); // view variable-set, constraint violations, indices,...
  
  cout << fixed;



 

  double t = 0.0;

  //possible bug-might not get to the xact target position
  //need to rectify
  double time_step = (solution.base_linear_->GetTotalTime() + 1e-5)/no_of_samples;
  //std::cout<<"\ntotal_Time:\t"<<solution.base_linear_->GetTotalTime() + 1e-5<<"\n";
int i =0;

  //while (t<=solution.base_linear_->GetTotalTime() + 1e-5)
  while(i<=no_of_samples)
  {
    if(t>2)
      t=2;
    //base angles to radian
    Eigen::Vector3d rad = solution.base_angular_->GetPoint(t).p();
    //if required to degree conversion from radian
    //rad =  rad/M_PI*180;

    for(int j=0;j<3;j++)
        {    

             (data+i)->base_linear[j] = solution.base_linear_->GetPoint(t).p().transpose()[j];
             
             
             (data+i)->base_angular[j] = rad[j];
             
             for (int k=0;k<n_ee;k++)//leg wise co ordinates
             {
             (data+i)->ee_linear[j+3*k] = solution.ee_motion_.at(k)->GetPoint(t).p().transpose()[j];
             (data+i)->ee_force[j+3*k] = solution.ee_force_.at(k)->GetPoint(t).p().transpose()[j];
              }
                    

        }
    //cout<<"\n"<<t<<"\n";
    i+=1;
    t+=time_step;

  }

//cout<<"\ni:"<<i<<"\n";

 if(towr_initial_output)
  {

      cout << "\n====================\nAnymal trajectory:\n====================\n";

   t= 0.0;
while (t<=solution.base_linear_->GetTotalTime() + 1e-5) {
    cout << "t=" << t << "\n";
    cout << "Base linear position x,y,z:   \t";
    cout << solution.base_linear_->GetPoint(t).p().transpose() << "\t[m]" << endl;

    cout << "Base Euler roll, pitch, yaw:  \t";
    Eigen::Vector3d rad = solution.base_angular_->GetPoint(t).p();
    cout << (rad/M_PI*180).transpose() << "\t[deg]" << endl;



   
    for (int ee_towr=0; ee_towr<n_ee; ++ee_towr) {
      cout<<"\n\tLeg: "<<ee_towr<<endl;

      cout << "\tFoot in Contact          \t";
      bool contact = solution.phase_durations_.at(ee_towr)->IsContactPhase(t);
      std::string foot_in_contact = contact? "yes" : "no";
      cout<< foot_in_contact<<endl;

      cout << "\tFoot position x,y,z:          \t";
      cout << solution.ee_motion_.at(ee_towr)->GetPoint(t).p().transpose() << "\t[m]" << endl;

      cout << "\tContact force x,y,z:          \t";
      cout << solution.ee_force_.at(ee_towr)->GetPoint(t).p().transpose() << "\t[N]" << endl;
      
      
      //solution.ee_motion_.at(ee_towr)->GetPoint(t);
      //solution.ee_force_.at(ee_towr)->GetPoint(t).p();
    }

   // cout << "Contact force x,y,z:          \t";
    //cout << solution.ee_force_.at(0)->GetPoint(t).p().transpose() << "\t[N]" << endl;

    //bool contact = solution.phase_durations_.at(0)->IsContactPhase(t);
    //std::string foot_in_contact = contact? "yes" : "no";
    //cout << "Foot in contact:              \t" + foot_in_contact << endl;

    cout << endl;

    t += time_step;
  }

}




}







extern "C"{

void Trajectory(Trajectory_data* data,float i_b_p[3],float i_p_a[3],float ee1[3],
                float ee2[3],float ee3[3],float ee4[3],float t[3],float t_a[3],int no_of_samples,int terrain_id,int gait_pattern)
{
  

   
  NlpFormulation formulation;
  SplineHolder solution;
  Eigen::Vector3d target,base_i_lin,base_i_ang,ee_1,ee_2,ee_3,ee_4,target_a;
  //std::cout <<"\nEnter Target Co-ordinates:\n";
  
  
  for(int i=0;i<3;i++)
  {
   target(i)=t[i];
   base_i_lin(i)=i_b_p[i];
   base_i_ang(i)=i_p_a[i];
   ee_1(i) = ee1[i];
   ee_2(i) = ee2[i];
   ee_3(i) = ee3[i];
   ee_4(i) = ee4[i];
   target_a(i) = t_a[i]; 

  }
  

  towr_trajectory(formulation,solution,
                  target,target_a,base_i_lin,base_i_ang,
                  ee_1,ee_2,ee_3,ee_4,
                  data,no_of_samples,terrain_id,gait_pattern);


}
}

/*

void towr_trajectory(NlpFormulation &formulation,SplineHolder &solution,
                    Eigen::Vector3d target,Eigen::Vector3d target_a,Eigen::Vector3d base_i_lin,
                    Eigen::Vector3d base_i_ang,Eigen::Vector3d ee1,
                    Eigen::Vector3d ee2,Eigen::Vector3d ee3,Eigen::Vector3d ee4,
                    Trajectory_data* data,int no_of_samples,int terrain_id,
                    int gait_pattern)
  */