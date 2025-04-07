/**
 * @file a_star.c
 * @brief Implementation of the A* algorithm.
 * TODO: Add details.
 *
 * @author YOUR NAME AND UNIVERSITY EMAIL/ID
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "priority_queue.h"
#include "a_star.h"

// Global graph
graph_node_t* graph[GRID_SIZE][GRID_SIZE];

// Heuristic function (Euclidean distance)
double heuristic(int x1, int y1, int x2, int y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// Initialize the graph
void init_graph() {
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            graph[y][x] = (graph_node_t*)malloc(sizeof(graph_node_t));
            graph[y][x]->x = x;
            graph[y][x]->y = y;
            graph[y][x]->g_cost = MAX_COST;
            graph[y][x]->h_cost = 0.0;
            graph[y][x]->f_cost = MAX_COST;
            graph[y][x]->parent = NULL;
            graph[y][x]->backward_parent = NULL;
            graph[y][x]->reached = 0;
            graph[y][x]->adj_list_head = NULL;
        }
    }
}

// Add an edge between two nodes
void add_edge(int x1, int y1, int x2, int y2, int distance) {
    adj_list_node_t* node1 = (adj_list_node_t*)malloc(sizeof(adj_list_node_t));
    adj_list_node_t* node2 = (adj_list_node_t*)malloc(sizeof(adj_list_node_t));
    
    node1->node = graph[y2][x2];
    node1->distance = distance;
    node1->next = graph[y1][x1]->adj_list_head;
    graph[y1][x1]->adj_list_head = node1;
    
    node2->node = graph[y1][x1];
    node2->distance = distance;
    node2->next = graph[y2][x2]->adj_list_head;
    graph[y2][x2]->adj_list_head = node2;
}

// Regular A* algorithm
void a_star(int start_x, int start_y, int end_x, int end_y) {
    PriorityQueue* open_set = create_priority_queue(100);
    PriorityQueue* closed_set = create_priority_queue(100);
    
    // Reset graph
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            graph[y][x]->g_cost = MAX_COST;
            graph[y][x]->h_cost = 0.0;
            graph[y][x]->f_cost = MAX_COST;
            graph[y][x]->parent = NULL;
            graph[y][x]->reached = 0;
        }
    }
    
    // Set start node
    graph[start_y][start_x]->g_cost = 0;
    graph[start_y][start_x]->h_cost = heuristic(start_x, start_y, end_x, end_y);
    graph[start_y][start_x]->f_cost = graph[start_y][start_x]->h_cost;
    graph[start_y][start_x]->reached = 1;
    
    enqueue(open_set, graph[start_y][start_x]);
    
    while (!is_empty(open_set)) {
        graph_node_t* current = dequeue(open_set);
        if (!current) break;
        
        if (current->x == end_x && current->y == end_y) {
            break;
        }
        
        enqueue(closed_set, current);
        
        adj_list_node_t* neighbor = current->adj_list_head;
        while (neighbor) {
            if (!is_in_queue(closed_set, neighbor->node)) {
                double tentative_g = current->g_cost + neighbor->distance;
                
                if (tentative_g < neighbor->node->g_cost) {
                    neighbor->node->parent = current;
                    neighbor->node->g_cost = tentative_g;
                    neighbor->node->h_cost = heuristic(neighbor->node->x, neighbor->node->y, end_x, end_y);
                    neighbor->node->f_cost = neighbor->node->g_cost + neighbor->node->h_cost;
                    
                    if (!is_in_queue(open_set, neighbor->node)) {
                        enqueue(open_set, neighbor->node);
                    }
                }
            }
            neighbor = neighbor->next;
        }
    }
    
    // Print path
    printf("\nPath from (%d,%d) to (%d,%d): ", start_x, start_y, end_x, end_y);
    graph_node_t* current = graph[end_y][end_x];
    while (current) {
        printf("(%d,%d) ", current->x, current->y);
        current = current->parent;
    }
    printf("\nTotal path cost: %.2f\n", graph[end_y][end_x]->g_cost);
    
    free_priority_queue(open_set);
    free_priority_queue(closed_set);
}

// PID Controller update function
void update_pid_controller(PIDController* pid, double current_error) {
    double error = current_error;
    double derivative = error - pid->last_error;
    pid->integral += error;
    
    // Anti-windup: limit integral term
    if (pid->integral > 5.0) pid->integral = 5.0;
    if (pid->integral < -5.0) pid->integral = -5.0;
    
    // Calculate weight adjustment
    double adjustment = pid->kp * error + 
                       pid->ki * pid->integral + 
                       pid->kd * derivative;
    
    // Update weight with bounds
    pid->weight = 1 + adjustment;
    if (pid->weight < 1.0) pid->weight = 1.0;
    if (pid->weight > 10.0) pid->weight = 10.0;
    
    pid->last_error = error;
}

// Dynamic Weighted A* with PID controller
void dynamic_weighted_a_star(int start_x, int start_y, int end_x, int end_y) {
    // Initialize PID controller
    PIDController pid = {0};
    pid.kp = 0.1;
    pid.ki = 0.01;
    pid.kd = 0.05;
    pid.weight = 1.0;
    pid.last_error = heuristic(start_x, start_y, end_x, end_y);
    pid.integral = 0.0;
    
    PriorityQueue* open_set = create_priority_queue(100);
    PriorityQueue* closed_set = create_priority_queue(100);
    
    // Reset graph
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            graph[y][x]->g_cost = MAX_COST;
            graph[y][x]->h_cost = 0.0;
            graph[y][x]->f_cost = MAX_COST;
            graph[y][x]->parent = NULL;
            graph[y][x]->reached = 0;
        }
    }
    
    // Set start node
    graph[start_y][start_x]->g_cost = 0;
    graph[start_y][start_x]->h_cost = heuristic(start_x, start_y, end_x, end_y);
    graph[start_y][start_x]->f_cost = graph[start_y][start_x]->h_cost;
    graph[start_y][start_x]->reached = 1;
    
    enqueue(open_set, graph[start_y][start_x]);
    
    printf("\nStarting PID A* search...\n");
    printf("Initial error: %.2f\n", graph[start_y][start_x]->h_cost);
    
    int iterations = 0;
    int max_iterations = 1000;
    
    while (!is_empty(open_set) && iterations < max_iterations) {
        iterations++;
        
        graph_node_t* current = dequeue(open_set);
        if (!current) break;
        
        if (current->x == end_x && current->y == end_y) {
            printf("Goal reached in %d iterations!\n", iterations);
            break;
        }
        
        double current_error = heuristic(current->x, current->y, end_x, end_y);
        update_pid_controller(&pid, current_error);
        
        enqueue(closed_set, current);
        
        adj_list_node_t* neighbor = current->adj_list_head;
        while (neighbor) {
            if (!is_in_queue(closed_set, neighbor->node)) {
                double tentative_g = current->g_cost + neighbor->distance;
                
                if (tentative_g < neighbor->node->g_cost) {
                    neighbor->node->parent = current;
                    neighbor->node->g_cost = tentative_g;
                    neighbor->node->h_cost = heuristic(neighbor->node->x, neighbor->node->y, end_x, end_y);
                    neighbor->node->f_cost = neighbor->node->g_cost + pid.weight * neighbor->node->h_cost;
                    
                    
                    if (!is_in_queue(open_set, neighbor->node)) {
                        enqueue(open_set, neighbor->node);
                    }
                }
            }
            neighbor = neighbor->next;
        }
        
        // Print PID values every 5 iterations
        if (iterations % 2 == 0) {
            printf("Iteration %d: weight=%.2f, error=%.2f\n", 
                   iterations, pid.weight, pid.last_error);
        }
    }
    
    if (iterations >= max_iterations) {
        printf("Maximum iterations reached!\n");
    }
    
    // Print path
    printf("\nFinal path from (%d,%d) to (%d,%d):\n", start_x, start_y, end_x, end_y);
    graph_node_t* current = graph[end_y][end_x];
    while (current) {
        printf("(%d,%d) ", current->x, current->y);
        current = current->parent;
    }
    printf("\nTotal path cost: %.2f\n", graph[end_y][end_x]->g_cost);
    
    free_priority_queue(open_set);
    free_priority_queue(closed_set);
}

// Bidirectional A* algorithm
void bidirectional_a_star(int start_x, int start_y, int end_x, int end_y) {
    init_queue_fwd();
    init_queue_bwd();
    
    // Reset graph 
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            graph[y][x]->g_cost = MAX_COST;
            graph[y][x]->h_cost = 0.0;
            graph[y][x]->f_cost = MAX_COST;
            graph[y][x]->parent = NULL;
            graph[y][x]->backward_parent = NULL;
            graph[y][x]->reached = 0;  // 0 = unvisited, 1 = forward, 2 = backward
        }
    }
    
    graph[start_y][start_x]->g_cost = 0;
    graph[start_y][start_x]->h_cost = heuristic(start_x, start_y, end_x, end_y);
    graph[start_y][start_x]->f_cost = graph[start_y][start_x]->h_cost;
    graph[start_y][start_x]->reached = 1;  // Forward
    
    graph[end_y][end_x]->g_cost = 0;
    graph[end_y][end_x]->h_cost = heuristic(end_x, end_y, start_x, start_y);
    graph[end_y][end_x]->f_cost = graph[end_y][end_x]->h_cost;
    graph[end_y][end_x]->reached = 2;  // Backward
    
    insert_fwd(graph[start_y][start_x], graph[start_y][start_x]->f_cost, &queue_fwd, &queue_size_fwd, &max_queue_size_fwd);
    insert_bwd(graph[end_y][end_x], graph[end_y][end_x]->f_cost, &queue_bwd, &queue_size_bwd, &max_queue_size_bwd);
    
    graph_node_t* intersection = NULL;
    double best_path_cost = MAX_COST;
    
    while (!is_empty_fwd() && !is_empty_bwd()) {
        // Forward search
        graph_node_t* current_fwd = get_fwd(queue_fwd, &queue_size_fwd);
        if (!current_fwd) break;
        
        adj_list_node_t* neighbor = current_fwd->adj_list_head;
        while (neighbor) {
            double tentative_g = current_fwd->g_cost + neighbor->distance;
            if (tentative_g < neighbor->node->g_cost) {
                neighbor->node->parent = current_fwd;
                neighbor->node->g_cost = tentative_g;
                neighbor->node->h_cost = heuristic(neighbor->node->x, neighbor->node->y, end_x, end_y);
                neighbor->node->f_cost = tentative_g + neighbor->node->h_cost;
                
                if (neighbor->node->reached == 2) {  // Found intersection
                    double total_cost = tentative_g + neighbor->node->g_cost;
                    if (total_cost < best_path_cost) {
                        best_path_cost = total_cost;
                        intersection = neighbor->node;
                    }
                } else if (neighbor->node->reached == 0) {
                    neighbor->node->reached = 1;  // Mark as forward
                    insert_fwd(neighbor->node, neighbor->node->f_cost, &queue_fwd, &queue_size_fwd, &max_queue_size_fwd);
                }
            }
            neighbor = neighbor->next;
        }
        
        // Backward search
        graph_node_t* current_bwd = get_bwd(queue_bwd, &queue_size_bwd);
        if (!current_bwd) break;
        
        neighbor = current_bwd->adj_list_head;
        while (neighbor) {
            double tentative_g = current_bwd->g_cost + neighbor->distance;
            if (tentative_g < neighbor->node->g_cost) {
                neighbor->node->backward_parent = current_bwd;  // Use backward_parent
                neighbor->node->g_cost = tentative_g;
                neighbor->node->h_cost = heuristic(neighbor->node->x, neighbor->node->y, start_x, start_y);
                neighbor->node->f_cost = tentative_g + neighbor->node->h_cost;
                
                if (neighbor->node->reached == 1) {  // Found intersection
                    double total_cost = tentative_g + neighbor->node->g_cost;
                    if (total_cost < best_path_cost) {
                        best_path_cost = total_cost;
                        intersection = neighbor->node;
                    }
                } else if (neighbor->node->reached == 0) {
                    neighbor->node->reached = 2;  // Mark as backward
                    insert_bwd(neighbor->node, neighbor->node->f_cost, &queue_bwd, &queue_size_bwd, &max_queue_size_bwd);
                }
            }
            neighbor = neighbor->next;
        }
    }
    
    if (intersection) {
        printf("\nIntersection found at (%d,%d)\n", intersection->x, intersection->y);
        
        // Print forward path (start to intersection)
        printf("Forward path (start to intersection): ");
        graph_node_t* curr = intersection;
        while (curr && curr != graph[start_y][start_x]) {
            printf("(%d,%d) ", curr->x, curr->y);
            curr = curr->parent;
        }
        printf("(%d,%d)\n", start_x, start_y);
        
        // Print backward path (intersection to goal)
        printf("Backward path (intersection to goal): ");
        curr = intersection;
        while (curr && curr != graph[end_y][end_x]) {
            printf("(%d,%d) ", curr->x, curr->y);
            curr = curr->backward_parent;
        }
        printf("(%d,%d)\n", end_x, end_y);
        
        printf("Total path cost: %.2f\n", best_path_cost);
    } else {
        printf("No path found!\n");
    }
    
    free_queue_fwd();
    free_queue_bwd();
}

// Main function
int main() {
    init_graph();
    
    // Create a simple grid with diagonal connections
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            // Connect to right neighbor
            if (x < GRID_SIZE - 1) {
                add_edge(x, y, x + 1, y, 1);
            }
            // Connect to bottom neighbor
            if (y < GRID_SIZE - 1) {
                add_edge(x, y, x, y + 1, 1);
            }
            // Connect to diagonal neighbors
            if (x < GRID_SIZE - 1 && y < GRID_SIZE - 1) {
                add_edge(x, y, x + 1, y + 1, 1);
            }
        }
    }
    
    printf("Testing regular A* from (0,0) to (9,9)\n");
    a_star(0, 0, 9, 9);
    
    printf("\nTesting bidirectional A* from (0,0) to (9,9)\n");
    bidirectional_a_star(0, 0, 9, 9);
    
    printf("\nTesting PID A* from (0,0) to (9,9)\n");
    dynamic_weighted_a_star(0, 0, 9, 9);
    
    return 0;
}






