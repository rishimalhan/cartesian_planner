///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUTHOR: RISHI MALHAN
// CENTER FOR ADVANCED MANUFACTURING
// UNIVERSITY OF SOUTHERN CALIFORNIA
// EMAIL: rmalhan@usc.edu
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <iostream>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <ros/package.h>
#include <robot_utilities/world_manager.hpp>
#include <robot_utilities/SerialLink_Manipulator.hpp>
#include <kdl/frames.hpp>
#include <robot_utilities/Data_Format_Mapping.hpp>

int main(){
    WM::WM wm;
    std::string urdf_path = ros::package::getPath("pct") + "/data/urdf/abb_irb_4600.urdf";
    KDL::Frame base_frame = KDL::Frame::Identity();
    KDL::Frame tcp_frame = KDL::Frame::Identity();
    std::string base_link = "base_link";
    std::string tip_link = "tool0";

    SerialLink_Manipulator::SerialLink_Manipulator robot(urdf_path, base_frame, tcp_frame, base_link, tip_link);

    std::string robot_obj = ros::package::getPath("pct") + "/data/robot_objs/irb4600/";
    wm.addRobot(robot_obj);

    Eigen::MatrixXd robot_T_part(4,4);
    robot_T_part << 1,0,0,0,
                    0,1,0,0,
                    0,0,1,0,
                    0,0,0,1;
    std::string wp_path = ros::package::getPath("pct") + "/data/meshes/fender_bin.stl";
    wm.addWorkpiece(wp_path, robot_T_part);
    Eigen::MatrixXd jt_config(6,1);
    jt_config << 0,0,0,0,0,0;
    std::vector<Eigen::MatrixXd> fk = robot.get_robot_FK_all_links( jt_config );
    wm.prepareSelfCollisionPatch(fk);
    
    for (int i=0; i<fk.size(); ++i)
        std::cout<< fk[i] << "\n";
    std::cout<< "Computing Collision\n";
    std::cout<< "Collision Status: " << wm.inCollision(fk) << "\n";
    std::cout<< "Penetration Depth: " << wm.getDistance(fk) << "\n";
    return 0;
}