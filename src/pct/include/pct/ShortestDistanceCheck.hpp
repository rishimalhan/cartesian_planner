#ifndef __SHORTESTDISTANCECHECK__HPP__
#define __SHORTESTDISTANCECHECK__HPP__

bool ShortestDistanceCheck(std::vector<Eigen::VectorXd> seg, ikHandler* ik_handler){
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
        
    // For all possible configurations, generate more segments
    if (ik_handler->solveIK(mid_x)){
        Eigen::VectorXd q1(6); // Configuration reached from seg[1]
        Eigen::VectorXd q2(6); // Configuration reached from seg[3]
        double dist1 = std::numeric_limits<double>::infinity();
        double dist2 = std::numeric_limits<double>::infinity();
        for (int i=0; i<ik_handler->solution.cols(); ++i){
            if ( (ik_handler->solution.col(i)-seg[1]).norm() < dist1 ){
                dist1 = (ik_handler->solution.col(i)-seg[1]).norm();
                q1 = ik_handler->solution.col(i);
            }

            if ( (ik_handler->solution.col(i)-seg[3]).norm() < dist2 ){
                dist2 = (ik_handler->solution.col(i)-seg[1]).norm();
                q2 = ik_handler->solution.col(i);
            }
        }
        if ( (q2-q1).norm() > 0.0 )
            return false;
        else
            return true;
    }
    else{
        // std::cout<< "BUGGGGG!!!! IK not feasible\n";
    }
    return false;
}

#endif 