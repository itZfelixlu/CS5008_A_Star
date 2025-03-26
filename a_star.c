/**
 * @file a_star.c
 * @brief Implementation of the A* algorithm.
 * TODO: Add details.
 *
 * @author YOUR NAME AND UNIVERSITY EMAIL/ID
 */

#include <stdio.h>
#include <math.h>

#include "priority_queue.h"
#include "a_star.h"

graph_node_t*** graph;

// TODO: GRAPH INIT, POSSIBLY RANDOM

void a_star(int start_x, int start_y, int end_x, int end_y) {
    graph[start_y][start_x]->cost = 0;
    graph[start_y][start_x]->prev = NULL;
    insert(graph[start_y][start_x], 0);
    while (!is_empty()) {
        graph_node_t* curr = get();
        if (curr->x == end_x && curr->y == end_y) {
            // Found target
            break;
        }
        adj_list_node_t* neighbor = curr->adj_list_head;
        while (neighbor != NULL) {
            int new_cost = curr->cost + neighbor->distance;
            if ((!neighbor->node->reached) || new_cost < neighbor->node->cost) {
                neighbor->node->cost = new_cost;
                neighbor->node->prev = curr;
                // Case 1: Basic Heuristic function using Eucliean Distance
                int priority = new_cost + heuristic(neighbor->node->x, neighbor->node->y, end_x, end_y); // TODO: ADD HEURISTIC VALUE WHEN FUNCTIONS IMPLEMENTED
                insert(neighbor->node, priority);
            }
        }
    }
    // At this point, the shortest path and distance should be obtainable from the end node
}

// TODO: Heuristic functions, free graph and adjacency list nodes
// Eucliean Distance for 8 directions
double heuristic(int x1, int y1, int x2, int y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}


void bidirectional_a_star(int start_x, int start_y, int end_x, int end_y) {
    // Initialize forward and backward queues
    init_queue_fwd();
    init_queue_bwd();

    // Start node (forward)
    graph_node_t* start = graph[start_y][start_x];
    start->cost = 0;
    start->prev = NULL;
    start->reached = 1;
    insert_fwd(start, 0, &queue_fwd, &queue_size_fwd, &max_queue_size_fwd);

    // Goal node (backward)
    graph_node_t* goal = graph[end_y][end_x];
    goal->cost_bwd = 0;  // Use cost_bwd for backward
    goal->prev_bwd = NULL;
    goal->reached = 2;
    insert_bwd(goal, 0, &queue_bwd, &queue_size_bwd, &max_queue_size_bwd);

    graph_node_t* intersection = NULL;

    while (!is_empty_fwd() && !is_empty_bwd()) {
        // Forward step
        graph_node_t* curr_fwd = get_fwd(queue_fwd, &queue_size_fwd);
        adj_list_node_t* neighbor_fwd = curr_fwd->adj_list_head;
        while (neighbor_fwd != NULL) {
            graph_node_t* n = neighbor_fwd->node;
            int new_cost = curr_fwd->cost + neighbor_fwd->distance;

            // Check for intersection
            if (n->reached == 2) {  // Visited by backward
                n->reached = 3;
                n->prev = curr_fwd;  // Forward parent
                n->cost = new_cost;  // Set forward cost
                intersection = n;
                goto cleanup;
            }

            if (n->reached == 0 || (n->reached == 1 && new_cost < n->cost)) {
                n->cost = new_cost;
                n->prev = curr_fwd;
                n->reached = 1;
                int priority = new_cost + heuristic(n->x, n->y, end_x, end_y);
                insert_fwd(n, priority, &queue_fwd, &queue_size_fwd, &max_queue_size_fwd);
            }
            neighbor_fwd = neighbor_fwd->next;
        }

        // Backward step
        graph_node_t* curr_bwd = get_bwd(queue_bwd, &queue_size_bwd);
        adj_list_node_t* neighbor_bwd = curr_bwd->adj_list_head;
        while (neighbor_bwd != NULL) {
            graph_node_t* n = neighbor_bwd->node;
            int new_cost = curr_bwd->cost_bwd + neighbor_bwd->distance;

            // Check for intersection
            if (n->reached == 1) {  // Visited by forward
                n->reached = 3;
                n->prev_bwd = curr_bwd;  // Backward parent
                n->cost_bwd = new_cost;  // Set backward cost
                intersection = n;
                goto cleanup;
            }

            if (n->reached == 0 || (n->reached == 2 && new_cost < n->cost_bwd)) {
                n->cost_bwd = new_cost;
                n->prev_bwd = curr_bwd;
                n->reached = 2;
                int priority = new_cost + heuristic(n->x, n->y, start_x, start_y);
                insert_bwd(n, priority, &queue_bwd, &queue_size_bwd, &max_queue_size_bwd);
            }
            neighbor_bwd = neighbor_bwd->next;
        }
    }

    cleanup:
    if (intersection) {
        printf("Intersection at (%d, %d)\n", intersection->x, intersection->y);
        reconstruct_path(intersection, start_x, start_y, end_x, end_y);
    } else {
        printf("No path found.\n");
    }
}

// Reconstruct and print the path for bidirectional A*
void reconstruct_path(graph_node_t* intersection, int start_x, int start_y, int end_x, int end_y) {
    if (!intersection) {
        printf("No path found.\n");
        return;
    }

    printf("Path from (%d, %d) to (%d, %d):\n", start_x, start_y, end_x, end_y);

    // Forward path: from start to intersection (reverse order)
    graph_node_t* curr = intersection;
    printf("Forward path: ");
    while (curr != NULL && (curr->x != start_x || curr->y != start_y)) {
        printf("(%d, %d) <- ", curr->x, curr->y);
        curr = curr->prev;
    }
    if (curr) {
        printf("(%d, %d)\n", curr->x, curr->y);  // Start node
    }

    // Backward path: from intersection to goal
    curr = intersection;
    printf("Backward path: (%d, %d) ", curr->x, curr->y);  // Intersection already printed
    while (curr != NULL && (curr->x != end_x || curr->y != end_y)) {
        curr = curr->prev_bwd;
        if (curr) {
            printf("-> (%d, %d) ", curr->x, curr->y);
        }
    }
    printf("\n");

    // Total cost
    int total_cost = intersection->cost + intersection->cost_bwd;
    printf("Total path cost: %d\n", total_cost);
}
    



