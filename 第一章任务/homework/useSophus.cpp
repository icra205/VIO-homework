#include <iostream>
#include <cmath>
using namespace std; 

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sophus/so3.h"
#include "sophus/se3.h"

int main( int argc, char** argv )
{
    // 沿Z轴转90度的旋转矩阵
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0,0,1)).toRotationMatrix();
    
    
    // 四元数乘法饶动微元
    Eigen::Matrix<double,4,1> w_q ;

    w_q << 0.005,0.01,0.015,1;


    // 李树饶动微元
    Eigen::Vector3d w;
    
    w << 0.01,0.02,0.03;


    // 生成李树与四元数
    Sophus::SO3 SO3_R(R);               // Sophus::SO(3)可以直接
    Eigen::Quaterniond q(R);            // 或者四元数

    cout<<"\n***********准备开始*******************\n"<<endl;

    cout<<endl;
    cout<<"R : \n"<<R<<endl;

    cout<<"******************************"<<endl;

    cout<<endl;
    cout<<"SO(3) from matrix: \n"<<SO3_R<<endl;

    cout<<endl;
    cout<<"q : \n"<<q.coeffs()<<endl;

    cout<<"************我是分割线*****************\n\n\n"<<endl;

    // 计算饶动后的数值
    Sophus::SO3 SO3_updated_R_left  = Sophus::SO3::exp(w)*SO3_R;
    Sophus::SO3 SO3_updated_R_right = SO3_R*Sophus::SO3::exp(w);
    

    cout<<"***********R饶动的值（李树）**********"<<endl;
    cout<<"**********R饶动的值（左）*********"<<endl;

    cout<<"SO3 updated = \n"<<SO3_updated_R_left<<endl;

    cout<<"**********R饶动的值（右）*********"<<endl;

    cout<<"SO3 updated = \n"<<SO3_updated_R_right<<endl;

    cout<<"***************************************\n\n\n"<<endl;
    
    /********************萌萌的分割线*****************************/
    cout<<"************我是分割线*****************"<<endl;
    
    //计算四元数的值
    Eigen::Quaterniond w_q_0(w_q);
    Eigen::Quaterniond q_updated_left;
    Eigen::Quaterniond q_updated_right;

    q_updated_left  = w_q_0 * q;
    q_updated_right = q * w_q_0;

    q_updated_left.normalize();
    q_updated_right.normalize();

    cout<<"***********四元数饶动的值*********\n"<<endl;
    cout<<"***四元数饶动的值（微变量）*******"<<endl;

    cout<<"w_q = \n"<<w_q_0.coeffs()<<endl;

    cout<<"\n***四元数饶动的值（结果zuo）******"<<endl;

    cout<<"q_updated = \n"<<q_updated_left.coeffs() <<endl;

    cout<<"***四元数饶动的值（结果you）*********"<<endl;

    cout<<"q_updated = \n"<<q_updated_right.coeffs()<<endl;

    cout<<"***************************************\n\n\n"<<endl;
    
    //传结果
    cout<<"************我是分割线*****************"<<endl;

    cout<<"***********jie        guo*********\n"<<endl;

    cout<<"***************************************"<<endl;

    cout<<"李树上（左） = \n"<<SO3_updated_R_left<<endl;

    cout<<"***************************************"<<endl;

    cout<<"李树上（右） = \n"<<SO3_updated_R_right<<endl;


    Sophus::SO3 SO3_q_l(q_updated_left );
    Sophus::SO3 SO3_q_r(q_updated_right);


    cout<<"\n***四元数饶动的值（结果zuo）******"<<endl;

    cout<<"q_左 = \n"<<SO3_q_l<<endl;

    cout<<"***四元数饶动的值（结果you）*********"<<endl;

    cout<<"q_右 = \n"<<SO3_q_r<<endl;

    cout<<"***************************************\n\n\n"<<endl;


    return 0;
}
