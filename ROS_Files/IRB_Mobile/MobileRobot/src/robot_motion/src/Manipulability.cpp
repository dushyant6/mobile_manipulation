#pragma once
#include <Eigen/Dense>
#include <memory>
#include <bits/stdc++.h>
#include <math.h>

class IRB_FK_IK{

public:
    std::vector<double> pickArmPose;
    std::vector<double> pickBasePose;
    double baseY;

    double getManipulability(double theta2, double theta5){
          double Manip = sqrt(
        2.6595e-08 * (pow(cos(theta2), 4) + 2 * pow(cos(theta2), 2) * pow(sin(theta2), 2) + pow(sin(theta2), 4)) 
        * 
        (
         22801 * pow(cos(theta2), 6) * pow(sin(theta5), 2) +
         51340 * pow(cos(theta2), 5) * sin(theta2) * pow(sin(theta5), 2) +
         74502 * pow(cos(theta2), 4) * pow(sin(theta2), 2) * pow(sin(theta5), 2) +
         102680 * pow(cos(theta2), 3) * pow(sin(theta2), 3) * pow(sin(theta5), 2) +
         80601 * pow(cos(theta2), 2) * pow(sin(theta2), 4) * pow(sin(theta5), 2) +
         51340 * cos(theta2) * pow(sin(theta2), 5) * pow(sin(theta5), 2) +
         28900 * pow(sin(theta2), 6) * pow(sin(theta5), 2)
        )
        );
        return Manip;
    }

    void getBasePose(std::vector<double> targetFrame){
        double objectHeight = targetFrame[2];
        double theta2LowerLim = -60;
        double theta2UpperLim = 60;
        //double theta2LowerLim = -60;
        //double theta2UpperLim = 60;

        double theta5LowerLim = -60;
        double theta5UpperLim = 60;

        double boxHeight = targetFrame[2];
        double eeTargetHeight = boxHeight - baseFrameHeight;

        double maxM = -100000;

        double theta2Target = 0;
        double theta5Target = 0;

        for(int i = theta2LowerLim; i < theta2UpperLim; i++){
            for(int j = theta5LowerLim; j < theta5UpperLim; j++){
                double eeHeight = 0.3400*cos(i*M_PI/180) - 0.3020*sin(i*M_PI/180) - 0.4020*cos(i*M_PI/180)*sin(j*M_PI/180) - 0.4020*cos(j*M_PI/180)*sin(i*M_PI/180) + 0.2900;
                if(abs(eeHeight - eeTargetHeight) < poseTolerance){
                    double M = getManipulability(i*M_PI/180, j*M_PI/180);
                    if(M>maxM){
                        maxM = M;
                        theta2Target = i*M_PI/180;
                        theta5Target = j*M_PI/180;
                    }



                }


            }
        }

        std::cout<<"Target theta2 = "<<theta2Target*180/M_PI<<std::endl;
        std::cout<<"Target theta5 = "<<theta5Target*180/M_PI<<std::endl;     
        std::cout<<"Max manipulability = "<<maxM<<std::endl;      

        pickArmPose = {0, theta2Target, 0, 0, theta5Target, 0};

        double ee_X = 0.3020*cos(theta2Target) + 0.3400*sin(theta2Target) + 0.2220*cos(theta2Target)*cos(theta5Target) - 0.2220*sin(theta2Target)*sin(theta5Target);
        double objectWidth = 0.04;
        baseY = targetFrame[1] - ee_X - objectWidth; //As the robot heads in positive X in grasp area
        std::cout<<"ee_X = "<<ee_X<<", baseY Pose = "<<baseY<<std::endl;
        pickBasePose = {targetFrame[0], baseY, 0};
    }

private:
    // Transformation matrix and jacobian calculation is done in the Matlab as it allows for easy use of symbolic variables
    // The file is present in matlab_jacobian directory for reference
    //Eigen::Matrix4d T7; //End effector transformation w.r.t. base frame
    
    double baseFrameHeight = 0.47;
    double poseTolerance = 0.01; //allowance for EE frame height to be compared to the object frame
    

    /*
    //arm angles
    double theta1 = 0; 
    double theta2 = 0;
    double theta3 = 0; 
    double theta4 = 0;
    double theta5 = 0;
    double theta6 = 0;

    Eigen::Matrix4d T7 {{cos(theta2)*sin(theta5) + cos(theta5)*sin(theta2),  0,   cos(theta2)*cos(theta5) - sin(theta2)*sin(theta5),  0.3020*cos(theta2) + 0.3400*sin(theta2) + 0.4020*cos(theta2)*cos(theta5) - 0.4020*sin(theta2)*sin(theta5)},
                          {0, -1, 0, 0},
                          {cos(theta2)*cos(theta5) - sin(theta2)*sin(theta5),  0, - cos(theta2)*sin(theta5) - cos(theta5)*sin(theta2), 0.3400*cos(theta2) - 0.3020*sin(theta2) - 0.4020*cos(theta2)*sin(theta5) - 0.4020*cos(theta5)*sin(theta2) + 0.2900},
                          {0, 0, 0, 1}
                        };
    
    Eigen::Matrix<double, 6, 6> J{
        {0, 0.3400*cos(theta2) - 0.3020*sin(theta2) - 0.0720*cos(theta2)*sin(theta5) - 0.0720*cos(theta5)*sin(theta2), 0.0700*cos(theta2) - 0.3020*sin(theta2) - 0.0720*cos(theta2)*sin(theta5) - 0.0720*cos(theta5)*sin(theta2), 0, - 0.0720*cos(theta2)*sin(theta5) - 0.0720*cos(theta5)*sin(theta2), 0},
        {0.3020*cos(theta2) + 0.3400*sin(theta2) + 0.0720*cos(theta2)*cos(theta5) - 0.0720*sin(theta2)*sin(theta5), 0, 0, cos(theta2)*(0.3020*sin(theta2) + 0.0720*cos(theta2)*sin(theta5) + 0.0720*cos(theta5)*sin(theta2)) - sin(theta2)*(0.3020*cos(theta2) + 0.0720*cos(theta2)*cos(theta5) - 0.0720*sin(theta2)*sin(theta5)), 0, (0.0720*cos(theta2)*sin(theta5) + 0.0720*cos(theta5)*sin(theta2))*(cos(theta2)*cos(theta5) - sin(theta2)*sin(theta5)) - (0.0720*cos(theta2)*cos(theta5) - 0.0720*sin(theta2)*sin(theta5))*(cos(theta2)*sin(theta5) + cos(theta5)*sin(theta2))},
        {0, 0.0720*sin(theta2)*sin(theta5) - 0.3400*sin(theta2) - 0.0720*cos(theta2)*cos(theta5) - 0.3020*cos(theta2), 0.0720*sin(theta2)*sin(theta5) - 0.0700*sin(theta2) - 0.0720*cos(theta2)*cos(theta5) - 0.3020*cos(theta2), 0, 0.0720*sin(theta2)*sin(theta5) - 0.0720*cos(theta2)*cos(theta5), 0},
        {0, 0, 0, cos(theta2), 0, cos(theta2)*cos(theta5) - sin(theta2)*sin(theta5)},
        {0, 1, 1, 0, 1, 0},
        {1, 0, 0, -sin(theta2), 0, -cos(theta2)*sin(theta5) - cos(theta5)*sin(theta2)}

    };

    //Manipulability = sqrt(det(J*transpoe(J))) //Refer to matlab file for derivation
    
    double M = sqrt(
        2.6595e-08 * (pow(cos(theta2), 4) + 2 * pow(cos(theta2), 2) * pow(sin(theta2), 2) + pow(sin(theta2), 4)) 
        * 
        (
         22801 * pow(cos(theta2), 6) * pow(sin(theta5), 2) +
         51340 * pow(cos(theta2), 5) * sin(theta2) * pow(sin(theta5), 2) +
         74502 * pow(cos(theta2), 4) * pow(sin(theta2), 2) * pow(sin(theta5), 2) +
         102680 * pow(cos(theta2), 3) * pow(sin(theta2), 3) * pow(sin(theta5), 2) +
         80601 * pow(cos(theta2), 2) * pow(sin(theta2), 4) * pow(sin(theta5), 2) +
         51340 * cos(theta2) * pow(sin(theta2), 5) * pow(sin(theta5), 2) +
         28900 * pow(sin(theta2), 6) * pow(sin(theta5), 2)
        )
        );
    */

};