///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUTHOR: RISHI MALHAN
// CENTER FOR ADVANCED MANUFACTURING
// UNIVERSITY OF SOUTHERN CALIFORNIA
// EMAIL: rmalhan@usc.edu
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////



#ifndef __graph_searches_hpp__
#define __graph_searches_hpp__

#include <pct/graph_description.hpp>
#include <limits>

namespace graph_searches{

    bool djikstra(boost_graph* graph, Eigen::VectorXi& path){
        std::cout<< "\n##############################################################\n";
        std::cout<< "Running Djikstra.........\n";

        dijkstra_shortest_paths(graph->g, graph->s,
                  predecessor_map(boost::make_iterator_property_map(graph->p.begin(), get(boost::vertex_index, graph->g))).
                  distance_map(boost::make_iterator_property_map(graph->d.begin(), get(boost::vertex_index, graph->g))));


        // for (int i=0; i<878; ++i){
        //     std::cout << "distance(" << i << ") = " << graph->d[i] << ", ";
        //     std::cout << "parent(" << i << ") = " << graph->p[i] << std::endl;
        // }
        // Check which leaf node has the lowest cost path
        double lowest_cost = std::numeric_limits<double>::infinity();
        int index;
        Eigen::VectorXi leaf_nodes = graph->leaf_nodes;
        bool leaf_connected = false;
        for (int i=0; i<leaf_nodes.size(); ++i){
            // std::cout<< "Cost: " << graph->d[ leaf_nodes(i) ] << "\n";
            if ( leaf_nodes(i)!=graph->p[leaf_nodes(i)] ) // if this node is not equal to its parent (terminal node)
                if (graph->d[ leaf_nodes(i) ] < lowest_cost){
                    index = i;
                    lowest_cost = graph->d[ leaf_nodes(i) ];
                    leaf_connected = true;
            }
        }

        if (!leaf_connected)
            return false;

        std::cout<< "Path Cost: " << graph->d[ leaf_nodes(index) ] << "\n";

        path.resize(graph->no_levels);
        int id = leaf_nodes(index);
        for(int row_no=graph->no_levels - 1; row_no>-1; --row_no){
            path(row_no) = id;
            id = graph->p[ id ];
        }
        // std::cout<< path.transpose() << "\n";
        std::cout<< "##############################################################\n";
        return true;
    };

};  


// std::cout << "distance(" << name[leaf_nodes[i]] << ") = " << graph->d[leaf_nodes[i]] << ", ";
// std::cout << "parent(" << name[leaf_nodes[i]] << ") = " << name[graph->p[leaf_nodes[i]]] << std::endl;

#endif