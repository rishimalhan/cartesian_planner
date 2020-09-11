#include <pct/path_consistency.hpp>

bool path_consistency_cvg(std::vector<Eigen::VectorXd> seg, ikHandler* ik_handler, int depth, double prev_dist){
    double max_distance = 0;

    std::cout<< "Max Distance of the Root: " << prev_dist << "\n";
    ik_handler->init_guess = seg[1];

    // In a loop keep sampling configurations and checking maximum error
    for (int no_samples = 1; no_samples <= 50; ++no_samples){
        Eigen::VectorXd dx = (seg[2] - seg[0]) / (no_samples+1);
        max_distance = 0.0;

        for (int i=0; i<=no_samples; ++i){
            // Generate two points for the segment
            std::vector<Eigen::VectorXd> sub_seg(4);
            sub_seg[0] = seg[0] + i*dx;
            sub_seg[2] = seg[0] + (i+1)*dx;

            // Make the Cartesian Axes Perpendicular. Evaluate // by = bz x bx and // bx = by x bz
            // by
            sub_seg[0](6) = (sub_seg[0](10)*sub_seg[0](5)) - (sub_seg[0](11)*sub_seg[0](4));
            sub_seg[0](7) = (sub_seg[0](11)*sub_seg[0](3)) - (sub_seg[0](9)*sub_seg[0](5));
            sub_seg[0](8) = (sub_seg[0](9)*sub_seg[0](4)) - (sub_seg[0](10)*sub_seg[0](3));
            // bx
            sub_seg[0](3) = (sub_seg[0](7)*sub_seg[0](11)) - (sub_seg[0](8)*sub_seg[0](10));
            sub_seg[0](4) = (sub_seg[0](8)*sub_seg[0](9)) - (sub_seg[0](6)*sub_seg[0](11));
            sub_seg[0](5) = (sub_seg[0](6)*sub_seg[0](10)) - (sub_seg[0](7)*sub_seg[0](9));
            sub_seg[0].segment(3,3) /= sub_seg[0].segment(3,3).norm(); //bx
            sub_seg[0].segment(9,3) /= sub_seg[0].segment(9,3).norm(); //by
            sub_seg[0].segment(6,3) /= sub_seg[0].segment(6,3).norm(); //bz

            // Make the Cartesian Axes Perpendicular. Evaluate // by = bz x bx and // bx = by x bz
            // by
            sub_seg[2](6) = (sub_seg[2](10)*sub_seg[2](5)) - (sub_seg[2](11)*sub_seg[2](4));
            sub_seg[2](7) = (sub_seg[2](11)*sub_seg[2](3)) - (sub_seg[2](9)*sub_seg[2](5));
            sub_seg[2](8) = (sub_seg[2](9)*sub_seg[2](4)) - (sub_seg[2](10)*sub_seg[2](3));
            // bx
            sub_seg[2](3) = (sub_seg[2](7)*sub_seg[2](11)) - (sub_seg[2](8)*sub_seg[2](10));
            sub_seg[2](4) = (sub_seg[2](8)*sub_seg[2](9)) - (sub_seg[2](6)*sub_seg[2](11));
            sub_seg[2](5) = (sub_seg[2](6)*sub_seg[2](10)) - (sub_seg[2](7)*sub_seg[2](9));
            sub_seg[2].segment(3,3) /= sub_seg[2].segment(3,3).norm(); //bx
            sub_seg[2].segment(9,3) /= sub_seg[2].segment(9,3).norm(); //by
            sub_seg[2].segment(6,3) /= sub_seg[2].segment(6,3).norm(); //bz


            // std::cout<< "Sample Start: " << sub_seg[0].segment(0,3).transpose() << "\n";
            // std::cout<< "Sample End: " << sub_seg[2].segment(0,3).transpose() << "\n";

            if (i==0) // First points
                sub_seg[1] = seg[1];
            else{
                ik_handler->solveIK(sub_seg[0]);
                sub_seg[1] = ik_handler->closest_sol;
            }

            if (i==no_samples) // Last point
                sub_seg[3] = seg[3];
            else{
                ik_handler->solveIK(sub_seg[2]);
                sub_seg[3] = ik_handler->closest_sol;
            }

            // std::cout<< "Sample Start: " << sub_seg[1].transpose() << "\n";
            // std::cout<< "Sample End: " << sub_seg[3].transpose() << "\n";

            // std::cout<< sub_seg[0].transpose() << "\n";
            // std::cout<< sub_seg[1].transpose() << "\n";
            // std::cout<< sub_seg[2].transpose() << "\n";
            // std::cout<< sub_seg[3].transpose() << "\n";
            double dist = get_dist(sub_seg, ik_handler);
            // std::cout<< "Distance: "<< dist << "\n";
            if (max_distance < dist){
                max_distance = dist;
            }
        }
        std::cout<< "Max Distance for no_samples: "<< no_samples << " is: " << max_distance << "\n";
    }
};

bool path_consistency_cvg(std::vector<Eigen::VectorXd> seg, ikHandler* ik_handler, double prev_dist){
    return path_consistency_cvg(seg, ik_handler, 0, prev_dist);
};


/*
0.975268 -0.31398   0.3204
0.975256 -0.32898 0.316738
0.975244 -0.34398 0.313075

Start Config: -0.412533  0.211855  0.541481 0.0334128   1.21386  -2.03505
End Config: -0.467046  0.264154  0.496211 0.0778333   1.26698  -2.13105
Max Distance: 0.000351303
Start Config: -0.412533  0.211855  0.541481 0.0334128   1.21386  -2.03505
End Config: -0.440207  0.237446  0.519256 0.0564198   1.23985   -2.0839
Max Distance: 8.92763e-05
Start Config: -0.440207  0.237446  0.519256 0.0564198   1.23985   -2.0839
End Config: -0.467046  0.264154  0.496211 0.0778333   1.26698  -2.13105
Max Distance: 8.64911e-05


*/