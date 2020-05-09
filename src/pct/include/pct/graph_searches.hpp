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

    bool djikstra(boost_graph* graph, Eigen::VectorXi& path){
        #ifdef SEARCH_ASSERT
        std::cout<< "\n##############################################################\n";
        std::cout<< "Running Djikstra.........\n";
        #endif
        dijkstra_shortest_paths(graph->g, graph->s,
                  predecessor_map(boost::make_iterator_property_map(graph->p.begin(), get(boost::vertex_index, graph->g))).
                  distance_map(boost::make_iterator_property_map(graph->d.begin(), get(boost::vertex_index, graph->g))));


        // Check which leaf node has the lowest cost path
        double lowest_cost = std::numeric_limits<double>::infinity();
        int index;
        Eigen::VectorXi leaf_nodes = graph->leaf_nodes;
        bool leaf_connected = false;
        Eigen::VectorXi curr_path(graph->no_levels);
        graph->paths.resize(graph->no_levels,1);
        #ifdef SEARCH_ASSERT
        std::cout<< "Cost:\n";
        #endif
        for (int i=0; i<leaf_nodes.size(); ++i){
            #ifdef SEARCH_ASSERT
            std::cout<< graph->d[ leaf_nodes(i) ] << ", ";
            #endif
            if ( leaf_nodes(i)!=graph->p[leaf_nodes(i)] ){ // if this node is not equal to its parent (terminal node)
                if (graph->d[ leaf_nodes(i) ] < lowest_cost){
                    index = i;
                    lowest_cost = graph->d[ leaf_nodes(i) ];
                    leaf_connected = true;
                }

                // Extras for research
                int id = leaf_nodes(i);
                for(int row_no=graph->no_levels - 1; row_no>-1; --row_no){
                    curr_path(row_no) = id;
                    id = graph->p[ id ];
                }
                graph->paths.col(graph->paths.cols()-1) = curr_path;
                graph->paths.conservativeResize(graph->no_levels,graph->paths.cols()+1);
                //
            }
        }
        graph->paths.conservativeResize(graph->no_levels,graph->paths.cols()-1); // Kill the last col
        #ifdef SEARCH_ASSERT
        std::cout<< "\n";
        #endif
        if (!leaf_connected){
            #ifdef SEARCH_ASSERT
            std::cout<< "##############################################################\n";
            #endif
            return false;
        }
        #ifdef SEARCH_ASSERT
        std::cout<< "Path Cost: " << graph->d[ leaf_nodes(index) ] << "\n";
        #endif

        path.resize(graph->no_levels);
        int id = leaf_nodes(index);
        for(int row_no=graph->no_levels - 1; row_no>-1; --row_no){
            path(row_no) = id;
            id = graph->p[ id ];
        }
        // std::cout<< path.transpose() << "\n";
        #ifdef SEARCH_ASSERT
        std::cout<< "##############################################################\n";
        #endif
        return true;
    };

};  


// std::cout << "distance(" << name[leaf_nodes[i]] << ") = " << graph->d[leaf_nodes[i]] << ", ";
// std::cout << "parent(" << name[leaf_nodes[i]] << ") = " << name[graph->p[leaf_nodes[i]]] << std::endl;

#endif