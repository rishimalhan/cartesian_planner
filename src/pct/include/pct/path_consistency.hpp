#ifndef __PATH_CONSISTENCY_HPP__
#define __PATH_CONSISTENCY_HPP__

double get_dist(std::vector<Eigen::VectorXd> seg, ikHandler* ik_handler){
    int no_samples = 10;
    Eigen::VectorXd dq = (seg[3] - seg[1])/no_samples;
    Eigen::VectorXd dx = (seg[2] - seg[0])/no_samples; // xyzbxbybz
    std::vector<double> dist; dist.clear();
    #ifdef DEBUG_PATH_CONSISTENCY
    std::cout<< "\n";
    #endif
    // double sum = 0;
    for (int i=0; i<=no_samples; ++i){
        KDL::JntArray theta = DFMapping::Eigen_to_KDLJoints(seg[1]+i*dq);
        KDL::Frame fk_kdl;
        ik_handler->robot->FK_KDL_TCP(theta,fk_kdl); // Tool tip
        Eigen::MatrixXd fk = DFMapping::KDLFrame_to_Eigen(fk_kdl);
        #ifdef DEBUG_PATH_CONSISTENCY    
        std::cout<< fk.block(0,3,3,1).transpose() << "\n";
        #endif
        dist.push_back( (fk.block(0,3,3,1) - (seg[0].segment(0,3)+i*dx.segment(0,3)) ).norm() ); // Should also include orientation
        // sum += (fk.block(0,3,3,1) - (seg[0].segment(0,3)+i*dx.segment(0,3)) ).norm();
    }
    #ifdef DEBUG_PATH_CONSISTENCY
    std::cout<< "\n";
    // std::cout<< "Max Distance: " << *max_element(dist.begin(),dist.end()) << "\n";
    #endif
    return *max_element(dist.begin(),dist.end());
    // return sum;
}



bool path_consistency(std::vector<Eigen::VectorXd> seg, ikHandler* ik_handler, int depth, double prev_dist){
    double red_factor = 0.9;

    // Analyze this segment
    double max_distance;
    if (depth>0)
        max_distance = get_dist(seg, ik_handler);
    else
        max_distance = prev_dist;

    #ifdef DEBUG_PATH_CONSISTENCY
    std::cout<< "Depth: " << depth << ". Distance: " << max_distance*1000 << " mm. " << "Previous Dist: " << prev_dist*1000 << " mm.\n";
    #endif

    if (depth==1) // Max depth to go
        if (max_distance < prev_dist*red_factor) // If current error is less then half the prev
            return true;
        else
            return false;
    depth++;

    Eigen::VectorXd mid_x = (seg[0] + seg[2])/2;
    // Make the Cartesian Axes Perpendicular. Evaluate // by = bz x bx and // bx = by x bz
    // by
    mid_x(6) = (mid_x(10)*mid_x(5)) - (mid_x(11)*mid_x(4));
    mid_x(7) = (mid_x(11)*mid_x(3)) - (mid_x(9)*mid_x(5));
    mid_x(8) = (mid_x(9)*mid_x(4)) - (mid_x(10)*mid_x(3));
    // bx
    mid_x(3) = (mid_x(7)*mid_x(11)) - (mid_x(8)*mid_x(10));
    mid_x(4) = (mid_x(8)*mid_x(9)) - (mid_x(6)*mid_x(11));
    mid_x(5) = (mid_x(6)*mid_x(10)) - (mid_x(7)*mid_x(9));
    mid_x.segment(3,3) /= mid_x.segment(3,3).norm(); //bx
    mid_x.segment(9,3) /= mid_x.segment(9,3).norm(); //by
    mid_x.segment(6,3) /= mid_x.segment(6,3).norm(); //bz
        
    #ifdef DEBUG_PATH_CONSISTENCY
    std::cout<< "MidX: " << mid_x.transpose() << "\n";
    std::cout<< "\n";
    #endif
    // For all possible configurations, generate more segments
    if (ik_handler->solveIK(mid_x)){
        std::vector<Eigen::VectorXd> seg1 = seg;
        std::vector<Eigen::VectorXd> seg2 = seg;
        for (int i=0; i<ik_handler->solution.cols(); ++i){
            seg1[2] = mid_x;
            seg1[3] = ik_handler->solution.col(i);
            seg2[0] = mid_x;
            seg2[1] = ik_handler->solution.col(i);
            bool status1 = path_consistency(seg1,ik_handler,depth,max_distance);
            bool status2 = path_consistency(seg2,ik_handler,depth,max_distance);
            if (status1 && status2 )
                return true; // Both configurations are path consistent
            #ifdef DEBUG_PATH_CONSISTENCY
            std::cout<< "\n";
            #endif
        }
    }
    else{
        // std::cout<< "BUGGGGG!!!! IK not feasible\n";
    }
    return false;
}

bool path_consistency(std::vector<Eigen::VectorXd> seg, ikHandler* ik_handler, double prev_dist){
    return path_consistency(seg, ik_handler, 0, prev_dist);
}

#endif 