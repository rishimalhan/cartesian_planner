///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUTHOR: RISHI MALHAN
// CONTRIBUTORS: ANIRUDDHA SHEMBEKAR, BRUAL SHAH
// CENTER FOR ADVANCED MANUFACTURING
// UNIVERSITY OF SOUTHERN CALIFORNIA
// EMAIL: rmalhan0112@gmail.com
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////



// Function generates a hatching path given a STL and generation parameters. 
// Output is a tree which comprises of connection between waypoints including tolerances. 
// First element in the list is a set of root nodes. 
// All the other elements form identical tree structures from each root node. 

#ifndef __GEN_CVRG_PLAN_HPP__
#define __GEN_CVRG_PLAN_HPP__

#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <gen_utilities/NPAM_utilities.hpp>
#include <igl/readSTL.h>
#include <robot_utilities/file_rw.hpp>
#include <robot_utilities/transformation_utilities.hpp>
#include <gen_utilities/utilities.hpp>

static void removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove)
{
    unsigned int numRows = matrix.rows()-1;
    unsigned int numCols = matrix.cols();

    if( rowToRemove < numRows )
        matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) = matrix.block(rowToRemove+1,0,numRows-rowToRemove,numCols);

    matrix.conservativeResize(numRows,numCols);
}

static Eigen::MatrixXd load_plan(std::string file_name){
    Eigen::MatrixXd tool_path = file_rw::file_read_mat(file_name); // Pre computed path file


    // new path is to keep alternate
    Eigen::MatrixXd new_path;
    int ctr=0;
    for (int i=0; i<tool_path.rows(); ++i){
        if (i%2==0){
            new_path.conservativeResize(ctr+1,12);
            new_path.row(ctr) = tool_path.row(i);
            ctr++;
        }
    }
    tool_path = new_path;


    // // Linearly interpolate more points within waypoints
    // Eigen::MatrixXd new_path;
    // int no_samples = 10;
    // int ctr = 0;
    // for (int i=0; i<tool_path.rows()-1; ++i){
    //     Eigen::VectorXd curr_pt = tool_path.row(i).transpose();
    //     Eigen::VectorXd nxt_pt = tool_path.row(i+1).transpose();
    //     Eigen::VectorXd dx = (nxt_pt-curr_pt)/no_samples;
    //     for (int j=0; j<=no_samples; ++j){
    //         new_path.conservativeResize(ctr+1,12);
    //         new_path.row(ctr) = tool_path.row(i) + j*dx.transpose();
    //         ctr++;
    //     }
    // }
    // tool_path = new_path;



    // Normalizing bxbybz and Evaluating by = bz x bx and bx = by x bz
    for (int i=0; i<tool_path.rows(); ++i){
        // Find by
        Eigen::Vector3d bz = tool_path.block(i,9,1,3).transpose();
        Eigen::Vector3d bx = tool_path.block(i,3,1,3).transpose();
        tool_path.block(i,6,1,3) = bz.cross(bx).transpose();
        // Find bx
        Eigen::Vector3d by = tool_path.block(i,6,1,3).transpose();
        tool_path.block(i,3,1,3) = by.cross(bz).transpose();
        // Normalize
        tool_path.block(i,3,1,3) /= tool_path.block(i,3,1,3).norm();
        tool_path.block(i,9,1,3) /= tool_path.block(i,9,1,3).norm();
        tool_path.block(i,6,1,3) /= tool_path.block(i,6,1,3).norm();
    }

    ROS_INFO("tool_path size: %d", (int)tool_path.rows());

    // Apply transformation from ros parameter server
    std::vector<double> tf; tf.clear();
    if(!ros::param::get("/cvrg_tf_param/world_T_part",tf))
        std::cout<< "Unable to Obtain part tf\n";
    Eigen::VectorXd tf_eigen(6);
    tf_eigen<< tf[0],tf[1],tf[2],tf[3],tf[4],tf[5];
    Eigen::Matrix4d world_T_part = Eigen::Matrix4d::Identity();
    world_T_part.block(0,0,3,3) = rtf::eul2rot(tf_eigen.segment(3,3).transpose(),"XYZ");
    world_T_part.block(0,3,3,1) = tf_eigen.segment(0,3);
    tool_path = rtf::apply_transformation_to_waypoints(tool_path,world_T_part);

    // Compute Average distance between each point
    double avg_dist = 0;
    for(int i=0; i<tool_path.rows()-1;++i){
        avg_dist += (tool_path.row(i+1).block(0,0,1,3)-tool_path.row(i).block(0,0,1,3)).norm();
    }
    avg_dist /= tool_path.rows()-1;
    std::cout<< "Avg Distance between Points is: " << avg_dist*1000 << "mm\n";

    std::string cvrg_path;
    if(!ros::param::get("/cvrg_file_paths/cvrg_path",cvrg_path))
        ROS_INFO("Could not find path file to save the waypoints");
    file_rw::file_write(cvrg_path,tool_path);

    return tool_path;

}

// STANDARD FOR WAYPOINT: Z AXIS IS NORMAL. X AXIS IS DIRECTION OF HEADING OF TOOL. Y AXIS IS COMPUTED.
static Eigen::MatrixXd gen_cvrg_plan(){
    ROS_INFO("COMPUTING SCANNING TOOL PATH......");
    bool scan_ori;
    bool scan_normals;
    double start_hatch_angle;
    int skip_hatching_lines;
    int skip_points_hatching_lines;
    int skip_points_orthogonal_lines;
    double outer_boundary_subtraction_x_dir;
    double outer_boundary_subtraction_y_dir;
    double z_height_offset_all_waypoints;

    if(!ros::param::get("/cvrg_path_parameters/scan_ori", scan_ori))
        ROS_INFO("FAIL TO LOAD PARAMETER: scan_ori");

    if(!ros::param::get("/cvrg_path_parameters/scan_normals", scan_normals))
        ROS_INFO("FAIL TO LOAD PARAMETER: scan_normals");

    if(!ros::param::get("/cvrg_path_parameters/start_hatch_angle", start_hatch_angle))
        ROS_INFO("FAIL TO LOAD PARAMETER: start_hatch_angle");
    
    if(!ros::param::get("/cvrg_path_parameters/skip_hatching_lines", skip_hatching_lines))
        ROS_INFO("FAIL TO LOAD PARAMETER: skip_hatching_lines");

    if(!ros::param::get("/cvrg_path_parameters/skip_points_hatching_lines", skip_points_hatching_lines))
        ROS_INFO("FAIL TO LOAD PARAMETER: skip_points_hatching_lines");

    if(!ros::param::get("/cvrg_path_parameters/skip_points_orthogonal_lines", skip_points_orthogonal_lines))
        ROS_INFO("FAIL TO LOAD PARAMETER: skip_points_orthogonal_lines");

    if(!ros::param::get("/cvrg_path_parameters/outer_boundary_subtraction_x_dir", outer_boundary_subtraction_x_dir))
        ROS_INFO("FAIL TO LOAD PARAMETER: outer_boundary_subtraction_x_dir");

    if(!ros::param::get("/cvrg_path_parameters/outer_boundary_subtraction_y_dir", outer_boundary_subtraction_y_dir))
        ROS_INFO("FAIL TO LOAD PARAMETER: outer_boundary_subtraction_y_dir");

    if(!ros::param::get("/cvrg_path_parameters/z_height_offset_all_waypoints", z_height_offset_all_waypoints))
        ROS_INFO("FAIL TO LOAD PARAMETER: z_height_offset_all_waypoints");


    outer_boundary_subtraction_x_dir *= 1000;
    outer_boundary_subtraction_y_dir *= 1000;

    ///////////////////////////////////////////////////////////////////////
    // DO NOT CHANGE THESE PARAMS THAT ARE BELOW
    // JUST ADJUST THE PATH WITH THE PARAMS MENTIONED ABOVE IN THE YAML FILE
    ///////////////////////////////////////////////////////////////////////

    // Gap between 2 hatching lines -
    double pathgap; // mm
    double pathgap_x; // mm
    double pathgap_y; // mm

    if(!ros::param::get("/cvrg_path_parameters/path_gap", pathgap))
        ROS_INFO("FAIL TO LOAD PARAMETER: path_gap");
    pathgap_x = pathgap*1000;
    pathgap_y = pathgap*1000;
    // grid addition (NOTE: higher number for higher aspect ratio of part in xy plane)
    double grid_addition = 0.2;
    
    // Path to generate -
    // 1 - Boundary
    // 2 - Hatching
    int Path_Number = 2;    // put number from above
    int FlipTravel = 0;     // 1 for yes, 0 for No

    ///////////////////////////
    // Bottom layer generation
    ///////////////////////////

    Eigen::MatrixXd v;
    Eigen::MatrixXd f;
    Eigen::MatrixXd n;

    std::string stl_path;
    ros::param::get("/cvrg_file_paths/mesh_path",stl_path);
    igl::readSTL(stl_path, v, f, n);

    // Convert to mm for simplicity
    v = v*1000;

    f = f.array()+1;    //NOTE : adding 1 make make start index as 1... this is compatible for rest of the code 
    Eigen::MatrixXd fillpts;

    ROS_INFO("STL V SIZE: %d, %d", (int)v.rows(), (int)v.cols());

    //////////////////////////////////////
    // xmax and ymax to get the grid size
    //////////////////////////////////////

    double xmax = v.block(0,0,v.rows(),1).maxCoeff() + grid_addition*1000;
    double ymax = v.block(0,1,v.rows(),1).maxCoeff() + grid_addition*1000;
    double xmin = v.block(0,0,v.rows(),1).minCoeff() - grid_addition*1000;
    double ymin = v.block(0,1,v.rows(),1).minCoeff() - grid_addition*1000;

    // double xmax = v.block(0,0,v.rows(),1).maxCoeff() - outer_boundary_subtraction;
    // double ymax = v.block(0,1,v.rows(),1).maxCoeff() - outer_boundary_subtraction;
    // double xmin = v.block(0,0,v.rows(),1).minCoeff() + outer_boundary_subtraction;
    // double ymin = v.block(0,1,v.rows(),1).minCoeff() + outer_boundary_subtraction;

    //////////////////////////////////////
    // the below params are used for constraining tool path such that
    // the tool diameter does not go outside the blade
    //////////////////////////////////////
    double xmax_boundary = v.block(0,0,v.rows(),1).maxCoeff();
    double ymax_boundary = v.block(0,1,v.rows(),1).maxCoeff();
    double xmin_boundary = v.block(0,0,v.rows(),1).minCoeff();
    double ymin_boundary = v.block(0,1,v.rows(),1).minCoeff();

    ROS_INFO("Bounds: %d, %d, %d, %d", (int)xmax, (int)ymax, (int)xmin, (int)ymin);
    //////////////////////////////////////////////////////////////////////////////
    // finding thickness of shell part and number of layers required for printing
    //////////////////////////////////////////////////////////////////////////////

    int num_of_layers = 1;

    ROS_INFO("Generating Grid Points...");
    std::vector<double> bounding_box_x(2);
    std::vector<double> bounding_box_y(2);
    ros::param::get("/cvrg_path_parameters/bounding_box_x",bounding_box_x);
    ros::param::get("/cvrg_path_parameters/bounding_box_y",bounding_box_y);
    //////////////////////////////////////////////////////
    // identifying the co-ordinates of bottom layer first
    //////////////////////////////////////////////////////
    
    Eigen::MatrixXd fnew;
    fnew = NPAM::identify_top_layer(v,f,n); 
    ////////////////////////////
    // generating the tool path
    ////////////////////////////

    Eigen::MatrixXd pts;
    double hatch_angle;
    double x_avg;
    double y_avg;
    long int tool_path_strt_idx = 0;
    Eigen::MatrixXd tool_path(1,1);

    for (int layer=1;layer<=num_of_layers;++layer)
    {
        ROS_INFO("GENERATING A LAYER");
        hatch_angle = start_hatch_angle;
        pts = NPAM::generate_grid_points(pathgap_x,pathgap_y,xmin,ymin,xmax,ymax,hatch_angle);


        // apply rotation to points
        x_avg = pts.block(0,0,pts.rows(),1).sum()/pts.rows();
        y_avg = pts.block(0,1,pts.rows(),1).sum()/pts.rows();
        Eigen::MatrixXd rotated_pts = NPAM::rotate_pts(pts,hatch_angle,x_avg,y_avg);
        

        ////////////////////////////
        // project grid points      
        ////////////////////////////
        fillpts = NPAM::project_grid_points(fnew, v, rotated_pts, hatch_angle, x_avg, y_avg);
        

        //////////////////////////////////////
        // the below params are used for constraining tool path such that
        // the tool diameter does not go outside the blade
        //////////////////////////////////////
        double xmax_boundary = fillpts.block(0,0,fillpts.rows(),1).maxCoeff() - outer_boundary_subtraction_x_dir;
        double ymax_boundary = fillpts.block(0,1,fillpts.rows(),1).maxCoeff()- outer_boundary_subtraction_y_dir;
        double xmin_boundary = fillpts.block(0,0,fillpts.rows(),1).minCoeff() + outer_boundary_subtraction_x_dir;
        double ymin_boundary = fillpts.block(0,1,fillpts.rows(),1).minCoeff() + outer_boundary_subtraction_y_dir;

        ROS_INFO("Boundary Bounds: %d, %d, %d, %d", (int)xmin_boundary, (int)xmax_boundary, (int)ymin_boundary, (int)ymax_boundary);

        ROS_INFO("Truncating Path for Given Boundary distances...");

        for(int row_id=fillpts.rows()-1; row_id > -1; row_id--)
        {
            double val_x = fillpts.coeff(row_id,0);
            double val_y = fillpts.coeff(row_id,1);
            // std::cout << "PT: " << val_x << " , " << val_y << ", >> [" << xmin_boundary << "," << xmax_boundary << "] ; [" << ymin_boundary << "," << ymax_boundary << " ]"<< std::endl;

            bool remove_current_row = false;
            if(val_y < ymin_boundary)
            {
                // std::cout << "Y LOW: " << row_id << std::endl;
                remove_current_row = true;
            }
            if(val_y > ymax_boundary)
            {
                // std::cout << "Y HIGH: " << row_id << std::endl;
                remove_current_row = true;
            }
            if(val_x < xmin_boundary)
            {
                // std::cout << "X LOW: " << row_id << std::endl;
                remove_current_row = true;
            }
            if(val_x > xmax_boundary)
            {
                // std::cout << "X HIGH: " << row_id << std::endl;
                remove_current_row = true;
            }
            if(remove_current_row)
            {
                removeRow(fillpts,row_id); // Remove the row in which the point belongs
            }
        }

        ////////////////////////////
        // infill path
        ////////////////////////////
        Eigen::MatrixXd layer_tool_path = NPAM::Infill_Path_with_bxbybz(fillpts, FlipTravel, skip_points_hatching_lines+1, hatch_angle, x_avg, y_avg, skip_hatching_lines, skip_points_orthogonal_lines+1);
        

        // Hack. Only works while generating one layer    
        tool_path.conservativeResize(layer_tool_path.rows(),layer_tool_path.cols());
        tool_path.block(0,0,layer_tool_path.rows(),layer_tool_path.cols()) = layer_tool_path;

        // // Inverting Tool Normals
        // layer_tool_path.col(9) = layer_tool_path.col(9).array()*(-1.0);
        // layer_tool_path.col(10) = layer_tool_path.col(10).array()*(-1.0);
        // layer_tool_path.col(11) = layer_tool_path.col(11).array()*(-1.0);

        // ////////////////////////////////
        // // Making continunous tool_path:
        // // adding start and end points
        // ////////////////////////////////
        // tool_path.conservativeResize(layer_tool_path.rows()+tool_path_strt_idx+0,layer_tool_path.cols());
        // tool_path.block(tool_path_strt_idx,0,(layer_tool_path.rows()+0),layer_tool_path.cols()) = Eigen::MatrixXd::Constant((layer_tool_path.rows()+0),layer_tool_path.cols(),0);
        // tool_path.block(tool_path_strt_idx,0,layer_tool_path.rows(),layer_tool_path.cols()) = layer_tool_path;
        // tool_path_strt_idx = tool_path_strt_idx + layer_tool_path.rows();
    }

    ROS_INFO("tool_path size: %d", (int)tool_path.rows());

    // Convert back to metres
    tool_path.block(0,0,tool_path.rows(),3) = tool_path.block(0,0,tool_path.rows(),3)/1000;
    
    // // Linearly interpolate more points within waypoints
    // Eigen::MatrixXd new_path;
    // int no_samples = 10;
    // int ctr = 0;
    // for (int i=0; i<tool_path.rows()-1; ++i){
    //     Eigen::VectorXd curr_pt = tool_path.row(i).transpose();
    //     Eigen::VectorXd nxt_pt = tool_path.row(i+1).transpose();
    //     Eigen::VectorXd dx = (nxt_pt-curr_pt)/no_samples;
    //     for (int j=0; j<=no_samples; ++j){
    //         new_path.conservativeResize(ctr+1,12);
    //         new_path.row(ctr) = tool_path.row(i) + j*dx.transpose();
    //         ctr++;
    //     }
    // }
    // tool_path = new_path;


    // Normalizing bxbybz and Evaluating by = bz x bx and bx = by x bz
    for (int i=0; i<tool_path.rows(); ++i){
        // Find by
        Eigen::Vector3d bz = tool_path.block(i,9,1,3).transpose();
        Eigen::Vector3d bx = tool_path.block(i,3,1,3).transpose();
        tool_path.block(i,6,1,3) = bz.cross(bx).transpose();
        // Find bx
        Eigen::Vector3d by = tool_path.block(i,6,1,3).transpose();
        tool_path.block(i,3,1,3) = by.cross(bz).transpose();
        // Normalize
        tool_path.block(i,3,1,3) /= tool_path.block(i,3,1,3).norm();
        tool_path.block(i,9,1,3) /= tool_path.block(i,9,1,3).norm();
        tool_path.block(i,6,1,3) /= tool_path.block(i,6,1,3).norm();
    }
    
    // Apply transformation from ros parameter server
    std::vector<double> tf; tf.clear();
    if(!ros::param::get("/cvrg_tf_param/world_T_part",tf))
        std::cout<< "Unable to Obtain part tf\n";
    Eigen::VectorXd tf_eigen(6);
    tf_eigen<< tf[0],tf[1],tf[2],tf[3],tf[4],tf[5];
    Eigen::Matrix4d world_T_part = Eigen::Matrix4d::Identity();
    world_T_part.block(0,0,3,3) = rtf::eul2rot(tf_eigen.segment(3,3).transpose(),"XYZ");
    world_T_part.block(0,3,3,1) = tf_eigen.segment(0,3);
    tool_path = rtf::apply_transformation_to_waypoints(tool_path,world_T_part);

    // Compute Average distance between each point
    double avg_dist = 0;
    for(int i=0; i<tool_path.rows()-1;++i){
        avg_dist += (tool_path.row(i+1).block(0,0,1,3)-tool_path.row(i).block(0,0,1,3)).norm();
    }
    avg_dist /= tool_path.rows()-1;
    std::cout<< "Avg Distance between Points is: " << avg_dist*1000 << "mm\n";

    std::string cvrg_path;
    ros::param::get("/cvrg_file_paths/cvrg_path",cvrg_path);
    file_rw::file_write(cvrg_path,tool_path);

    return tool_path;
}
#endif