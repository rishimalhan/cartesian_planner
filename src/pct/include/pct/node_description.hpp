///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUTHOR: RISHI MALHAN
// CENTER FOR ADVANCED MANUFACTURING
// UNIVERSITY OF SOUTHERN CALIFORNIA
// EMAIL: rmalhan0112@gmail.com
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////



#ifndef __node_description_hpp__
#define __node_description_hpp__

#include <Eigen/Eigen>

struct node{
    int row_id;
    int id;
    int parent_id;
    int tcp_id;
    Eigen::VectorXd jt_config;
    double g_cost;
    int family_id;
    int depth; 
    Eigen::VectorXd wp; // xyzbxbybz
    Eigen::VectorXd wp_eul; //xyzrxryrz
    Eigen::VectorXd wp_qt; //xyzquat
    Eigen::VectorXd wp_quat; //Quaternion
    Eigen::VectorXd wp_ff; // xyzbxbybz of flange wrt rob_base
    Eigen::MatrixXd jacobian;
};


#endif