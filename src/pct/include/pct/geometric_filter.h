#ifndef GEOMETRIC_FILTER_H
#define GEOMETRIC_FILTER_H

#include <robot_utilities/world_manager.hpp>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <robot_utilities/transformation_utilities.hpp>
#include <pct/timer.hpp>
#include <iomanip>

class GeometricFilterHarness
{
    private:
        // Create Collision Checker
        WM::WM wm;

        //
        Eigen::Matrix4d ff_T_tcp_;
        Eigen::VectorXd world_T_ff;

        // Generate Waypoint xyz_bxbybz from transform
        Eigen::VectorXd generate_xyz_rxryrz_from_transform(Eigen::Matrix4d transform){
            Eigen::VectorXd waypoint(12);
            waypoint.segment(0,3) = transform.block(0,3,3,1);
            waypoint.segment(3,3) = transform.block(0,0,3,1);
            waypoint.segment(6,3) = transform.block(0,1,3,1);
            waypoint.segment(9,3) = transform.block(0,2,3,1);
            return waypoint;
        }

        // Generate Eigen matrix from euler angles
        Eigen::Matrix4d generate_transform_from_xyz_rxryrz_(
            double const x,
            double const y,
            double const z,
            double const rot_x,
            double const rot_y,
            double const rot_z
        )
        {
            Eigen::VectorXd tf_eigen(6);
            tf_eigen<< x,y,z,rot_x,rot_y,rot_z;
            Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
            transform.block(0,0,3,3) = rtf::eul2rot(tf_eigen.segment(3,3).transpose(),"XYZ");
            transform.block(0,3,3,1) = tf_eigen.segment(0,3);
            return transform;
        }
        Eigen::Matrix4d generate_transform_from_xyz_bxbybz_(
            Eigen::VectorXd const& vec
        )
        {
            Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
            transform.block(0,3,3,1) = vec.segment(0,3);
            transform.block(0,0,3,1) = vec.segment(3,3);
            transform.block(0,1,3,1) = vec.segment(6,3);
            transform.block(0,2,3,1) = vec.segment(9,3);
            return transform;
        }
        bool is_collision_free_(Eigen::Matrix4d const& world_T_tcp,
                                Eigen::Matrix4d const& flange_frame_T_tcp,
                                bool check_coll
                                )// should be const, but WM is not const correct
        {
            // TODO:
            // Take the world_T_tool transform from rosparam 
            // Since tool origin and flange could have diff transform
            auto world_T_tool = world_T_tcp * flange_frame_T_tcp.inverse();
            world_T_ff = generate_xyz_rxryrz_from_transform(world_T_tool);
            if (!check_coll)
                return true;
            std::vector<Eigen::MatrixXd> tool_tf;
            tool_tf.push_back(world_T_tool);
            auto ret_val = wm.inCollision(tool_tf);
            // ROS_WARN_STREAM("world_T_tool");
            // std::cout << world_T_tool << std::endl;
            // ROS_WARN_STREAM("world_T_tcp");
            // std::cout << world_T_tcp << std::endl;
            // ROS_WARN_STREAM("flange_frame_T_tcp");
            // std::cout << flange_frame_T_tcp << std::endl;
            // ROS_WARN_STREAM("flange_frame_T_tcp_inv");
            // std::cout << flange_frame_T_tcp.inverse() << std::endl;
            // std::cout<< "Collision Status: " << ret_val << "\n";
            return (ret_val == 0);
        }
    public:
        GeometricFilterHarness()
        {
            world_T_ff.resize(12);
            std::vector<double> tf_part(6);
            Eigen::VectorXd tf_eigen(6);
            // Mesh Path
            std::string wp_path;
            if(!ros::param::get("/cvrg_file_paths/mesh_path",wp_path)){
                ROS_WARN("Unable to Obtain mesh path");
                throw;
            }
            // Part to World tf: x,y,z,rx,ry,rz
            if(!ros::param::get("/cvrg_tf_param/world_T_part",tf_part)){
                ROS_WARN("Unable to Obtain part tf");
                throw;
            }
            // Path to folder where collision stl is located
            std::string tool_name;
            if(!ros::param::get("/tool_name",tool_name)){
                ROS_WARN("Unable to Obtain Tool name");
                throw;
            }
            std::string toolstl_folder;
            if(!ros::param::get("/"+tool_name + "/tool_stl_coll_folder",toolstl_folder)){
                ROS_WARN("Unable to Obtain tool collision stl folder");
                throw;
            }

            // Read in the ff_T_tcp transform
            std::vector<double> ff_T_tcp_vec(6);
            if(!ros::param::get("/"+tool_name + "/ff_T_tool",ff_T_tcp_vec)){
                ROS_ERROR("Unable to obtain ff_T_tool");
                throw;
            }
            ff_T_tcp_ = generate_transform_from_xyz_rxryrz_(
                                                ff_T_tcp_vec[0],
                                                ff_T_tcp_vec[1],
                                                ff_T_tcp_vec[2],
                                                ff_T_tcp_vec[3],
                                                ff_T_tcp_vec[4],
                                                ff_T_tcp_vec[5]
                                                );

            auto world_T_part = generate_transform_from_xyz_rxryrz_(
                                                tf_part[0],
                                                tf_part[1],
                                                tf_part[2],
                                                tf_part[3],
                                                tf_part[4],
                                                tf_part[5]
                                                );

            // Add tool as a robot
            wm.addRobot(toolstl_folder);
            // Add the workpiece
            wm.addWorkpiece(wp_path, world_T_part);
            // Any other env variables can be added here

            ROS_WARN_STREAM("GeometricFilterHarness initialized");
        }
        
        std::vector<Eigen::MatrixXd> generate_flange_frames(
                                   std::vector<Eigen::MatrixXd> const& waypoint_samples_all_levels,
                                   std::vector< Eigen::MatrixXd > const& tcp_list
                                   ) // should be const, but WM is not const correct
        {
            std::vector<Eigen::MatrixXd> flange_frames; flange_frames.clear();
            int const NUM_COLS = 12;
            bool should_take;
            int num_samples_counter = 0;
            int level_counter = 0;
            int no_valid_samples = 0;
            for (auto const& waypoint_samples_single_level : waypoint_samples_all_levels){
                Eigen::MatrixXd tmp;
                int counter = 0;
                for (auto const& tcp : tcp_list){
                    auto flange_frame_T_tcp = tcp;
                    // each level has multiple samples
                    for (auto row_id = 0; row_id < waypoint_samples_single_level.rows(); ++row_id){
                        auto const& waypoint_sample = waypoint_samples_single_level.row(row_id);
                        auto world_T_tcp = generate_transform_from_xyz_bxbybz_(
                            waypoint_sample
                        );
                        
                        #ifdef PCT_PLANNER
                        should_take = is_collision_free_(world_T_tcp, flange_frame_T_tcp,true);
                        #endif
                        #ifdef BASELINE
                        should_take = is_collision_free_(world_T_tcp, flange_frame_T_tcp,false);
                        #endif
                        if (should_take)
                        {
                            
                            tmp.conservativeResize(counter+1, NUM_COLS);
                            tmp.row(counter) = world_T_ff;
                            counter++;
                            no_valid_samples++;
                        }
                        num_samples_counter++;
                        // ROS_DEBUG_STREAM((should_take?"$":"X") << "----- when testing " << tcp_name << ", level " << level_counter << ", row_id " << row_id);    
                    }
                }
                flange_frames.push_back(tmp);
                level_counter++;
            }
            auto reduction_percent = 100 * (double)(num_samples_counter-no_valid_samples)/ num_samples_counter;
            ROS_INFO_STREAM(std::setw(25) << "Reduction: " << std::setw(20) << std::setprecision(4) << reduction_percent << " %");
            ROS_INFO_STREAM(std::setw(25) << "Accepted sample count: " << std::setw(20) << std::setprecision(4) << no_valid_samples);
            ROS_INFO_STREAM(std::setw(25) << "Total sample count: " << std::setw(20) << std::setprecision(4) << num_samples_counter);

            return flange_frames;
        }

};

#endif