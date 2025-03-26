/**
 * @file a_star.h
 * @brief Header file for the A* algorithm.
 * TODO: Add details.
 *
 * @author YOUR NAME AND UNIVERSITY EMAIL/ID
 */

/*
typedef enum space {
    START,
    CLEAR,
    OBSTACLE,
    END
} space_t;
*/

// Note: Every node in a grid is adjacent to its 8 neighbors, adj_list not required if distances are all the same (e.g. no terrain)
typedef struct adj_list_node {
    int distance;
    struct graph_node* node;
    struct adj_list_node* next;
} adj_list_node_t;

typedef struct graph_node {
    int x; // x-coordinate
    int y; // y-coordinate
    int reached; // Whether this node has been reached by the search or not, 0 = unvisited, 1 = forward, 2 = backward, 3 = both
    //space_t type; // Type of space that this node represents
    struct adj_list_node* adj_list_head; // Head of the adjacency list for this node's neighbors
    int cost; // Lowest distance to this node found thus far
    int cost_bwd; // Account for backward cost
    struct graph_node* prev; // Previous node in the lowest distance path to this node thus far
    struct graph_node* prev_bwd; 
} graph_node_t;
