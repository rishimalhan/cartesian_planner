///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUTHOR: RISHI MALHAN
// CENTER FOR ADVANCED MANUFACTURING
// UNIVERSITY OF SOUTHERN CALIFORNIA
// EMAIL: rmalhan@usc.edu
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////



#ifndef __node_description_hpp__
#define __node_description_hpp__

#include <Eigen/Eigen>

struct node{
    int id;
    int parent_id;
    int tcp_id;
    Eigen::VectorXd jt_config;
    double g_cost;
    // Optional depth and index corresponding to original wpTol
    int depth; 
    int index;
};


#endif