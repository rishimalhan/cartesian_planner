///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUTHOR: RISHI MALHAN
// CENTER FOR ADVANCED MANUFACTURING
// UNIVERSITY OF SOUTHERN CALIFORNIA
// EMAIL: rmalhan0112@gmail.com
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////



#ifndef __graph_searches_hpp__
#define __graph_searches_hpp__

#include <pct/graph_description.hpp>
#include <limits>

namespace graph_searches{

    bool djikstra(boost_graph* graph, Eigen::VectorXi& path, double& path_cost){
        if ( !(graph->root_connected && graph->leaf_connected) ){
            path_cost = std::numeric_limits<double>::infinity();
            return false;
        }

        #ifdef SEARCH_ASSERT
        std::cout<< "\n##############################################################\n";
        std::cout<< "Running Djikstra.........\n";
        #endif
        dijkstra_shortest_paths(graph->g, graph->s,
                  predecessor_map(boost::make_iterator_property_map(graph->p.begin(), get(boost::vertex_index, graph->g))).
                  distance_map(boost::make_iterator_property_map(graph->d.begin(), get(boost::vertex_index, graph->g))));
        path_cost = graph->d[ 1 ];
        path.resize(graph->no_levels);
        if (graph->p[ 1 ] == 1){
            path_cost = std::numeric_limits<double>::infinity();
            return false;
        }

        int id = 1;
        for (int row_no=graph->no_levels - 1; row_no>-1; --row_no){
            id = graph->p[ id ];
            path(row_no) = id;
        }

        #ifdef SEARCH_ASSERT
        std::cout<< "Path Cost: " << graph->d[ 1 ] << "\n";
        #endif

        #ifdef SEARCH_ASSERT
        std::cout<< "##############################################################\n";
        #endif

        return true;
    };

};  


// std::cout << "distance(" << name[leaf_nodes[i]] << ") = " << graph->d[leaf_nodes[i]] << ", ";
// std::cout << "parent(" << name[leaf_nodes[i]] << ") = " << name[graph->p[leaf_nodes[i]]] << std::endl;

#endif