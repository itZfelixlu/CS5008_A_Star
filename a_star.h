/**
 * @file a_star.h
 * @brief Header file for the A* algorithm.
 * TODO: Add details.
 *
 * @author YOUR NAME AND UNIVERSITY EMAIL/ID
 */

#ifndef A_STAR_H
#define A_STAR_H

#include <stddef.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "priority_queue.h"

#define GRID_SIZE 10
#define MAX_COST 1e9

// PID Controller structure
typedef struct {
    double kp;          // Proportional gain
    double ki;          // Integral gain
    double kd;          // Derivative gain
    double weight;      // Current weight
    double last_error;  // Last error value
    double integral;    // Integral term
} PIDController;

// Graph node structure
typedef struct graph_node {
    int x, y;
    double g_cost;      // Cost from start
    double h_cost;      // Heuristic cost to goal
    double f_cost;      // Total cost (g + h)
    struct graph_node* parent;        // Parent in forward search
    struct graph_node* backward_parent;  // Parent in backward search
    int reached;        // 0 = unvisited, 1 = forward, 2 = backward
    struct adj_list_node* adj_list_head;
} graph_node_t;

// Adjacency list node structure
typedef struct adj_list_node {
    graph_node_t* node;
    int distance;
    struct adj_list_node* next;
} adj_list_node_t;

// Function declarations
void init_graph();
void add_edge(int x1, int y1, int x2, int y2, int distance);
void a_star(int start_x, int start_y, int end_x, int end_y);
void bidirectional_a_star(int start_x, int start_y, int end_x, int end_y);
void dynamic_weighted_a_star(int start_x, int start_y, int end_x, int end_y);
void update_pid_controller(PIDController* pid, double current_error);
void reconstruct_path(graph_node_t* node, int start_x, int start_y);
void reconstruct_path_bidirectional(graph_node_t* intersection, int start_x, int start_y, int end_x, int end_y);

#endif // A_STAR_H
