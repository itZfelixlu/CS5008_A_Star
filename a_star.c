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
#include <stdbool.h>

#include "priority_queue.h"
#include "a_star.h"
#define MAX_SLOPE_REGIONS 5
#define MAX_OBSTACLES (GRID_SIZE * GRID_SIZE * 3) // Max possible blocked edges
// Global variables
graph_node_t* graph[GRID_SIZE][GRID_SIZE];
int nodes_explored = 0;

int slope_regions[MAX_SLOPE_REGIONS][4]; // [x_min, y_min, x_max, y_max]
int num_slope_regions = 0; // Track number used

int obstacles[MAX_OBSTACLES][2]; // [x, y]
int obstacle_count = 0;

// typedef struct {
//     graph_node_t* path;
//     int path_length;
//     double total_cost;
//     int nodes_explored;
// } path_result_t;

// Heuristic function (Euclidean distance)
double heuristic(int x1, int y1, int x2, int y2) {
    //return abs(x2 - x1) + abs(y2 - y1);
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
void add_edge(int x1, int y1, int x2, int y2, double distance) {
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

// Clear graph edges for reuse
void clear_graph_edges() {
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            adj_list_node_t* current = graph[y][x]->adj_list_head;
            while (current) {
                adj_list_node_t* temp = current;
                current = current->next;
                free(temp);
            }
            graph[y][x]->adj_list_head = NULL;
        }
    }
}

// Setup simple grid with uniform cost
void setup_simple_grid() {
    clear_graph_edges();
    obstacle_count = 0;
    num_slope_regions = 0;
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            if (x < GRID_SIZE - 1) {
                add_edge(x, y, x + 1, y, 1.0); // Right
            }
            if (y < GRID_SIZE - 1) {
                add_edge(x, y, x, y + 1, 1.0); // Down
            }
            if (x < GRID_SIZE - 1 && y < GRID_SIZE - 1) {
                add_edge(x, y, x + 1, y + 1, sqrt(2)); // Diagonal
            }
        }
    }
}

// Setup complicated grid with obstacles and slopes
void setup_complicated_grid() {
    clear_graph_edges();
    srand(1234);
    const double OBSTACLE_PROB = 0.1;
    obstacle_count = 0;
    num_slope_regions = 4;
    
    // Define slope regions (e.g., 3 random 15x15 patches)
    int region_size = 15;
    for (int i = 0; i < num_slope_regions; i++) {
        int x_min = rand() % (GRID_SIZE - region_size);
        int y_min = rand() % (GRID_SIZE - region_size);
        slope_regions[i][0] = x_min;
        slope_regions[i][1] = y_min;
        slope_regions[i][2] = x_min + region_size;
        slope_regions[i][3] = y_min + region_size;
    }
    
    // First pass: Mark cells as obstacles
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            // Skip start and end points
            if ((x == 0 && y == 0) || (x == GRID_SIZE - 1 && y == GRID_SIZE - 1)) {
                continue;
            }
            
            // Add obstacle with probability
            if ((double)rand() / RAND_MAX < OBSTACLE_PROB && obstacle_count < MAX_OBSTACLES) {
                obstacles[obstacle_count][0] = x;
                obstacles[obstacle_count][1] = y;
                obstacle_count++;
            }
        }
    }
    
    // Second pass: Add edges, considering obstacles and slopes
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            bool is_obstacle = false;
            // Check if current cell is an obstacle
            for (int i = 0; i < obstacle_count; i++) {
                if (obstacles[i][0] == x && obstacles[i][1] == y) {
                    is_obstacle = true;
                    break;
                }
            }
            if (is_obstacle) continue;
            
            bool in_slope = false;
            for (int i = 0; i < num_slope_regions; i++) {
                if (x >= slope_regions[i][0] && x <= slope_regions[i][2] &&
                    y >= slope_regions[i][1] && y <= slope_regions[i][3]) {
                    in_slope = true;
                    break;
                }
            }
            
            double cost = in_slope ? 3.0 : 1.0;
            
            // Add edges to adjacent cells if they're not obstacles
            if (x < GRID_SIZE - 1) {
                bool right_obstacle = false;
                for (int i = 0; i < obstacle_count; i++) {
                    if (obstacles[i][0] == x + 1 && obstacles[i][1] == y) {
                        right_obstacle = true;
                        break;
                    }
                }
                if (!right_obstacle) {
                    add_edge(x, y, x + 1, y, cost);
                }
            }
            
            if (y < GRID_SIZE - 1) {
                bool down_obstacle = false;
                for (int i = 0; i < obstacle_count; i++) {
                    if (obstacles[i][0] == x && obstacles[i][1] == y + 1) {
                        down_obstacle = true;
                        break;
                    }
                }
                if (!down_obstacle) {
                    add_edge(x, y, x, y + 1, cost);
                }
            }
            
            if (x < GRID_SIZE - 1 && y < GRID_SIZE - 1) {
                bool diag_obstacle = false;
                for (int i = 0; i < obstacle_count; i++) {
                    if (obstacles[i][0] == x + 1 && obstacles[i][1] == y + 1) {
                        diag_obstacle = true;
                        break;
                    }
                }
                if (!diag_obstacle) {
                    add_edge(x, y, x + 1, y + 1, cost * sqrt(2));
                }
            }
        }
    }
}

// Regular A* algorithm
path_result_t a_star(int start_x, int start_y, int end_x, int end_y) {
    path_result_t regular_result = {0};
    nodes_explored = 0;  // Reset counter at start

    //Initialize priority queues
    PriorityQueue* open_set = create_priority_queue(1000);
    PriorityQueue* closed_set = create_priority_queue(1000);
    
    // Allocate memory for explored nodes (we'll resize if needed)
    regular_result.explored_nodes = (graph_node_t*)malloc(1000 * sizeof(graph_node_t));
    
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
    nodes_explored = 1;  // Start with 1 for the start node
    
    // Store start node in explored nodes
    regular_result.explored_nodes[0].x = start_x;
    regular_result.explored_nodes[0].y = start_y;
    
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
                        nodes_explored++;
                        
                        // Store explored node
                        if (nodes_explored >= 1000) {
                            // Resize the array if needed
                            regular_result.explored_nodes = realloc(regular_result.explored_nodes, 
                                (nodes_explored + 1000) * sizeof(graph_node_t));
                        }
                        regular_result.explored_nodes[nodes_explored - 1].x = neighbor->node->x;
                        regular_result.explored_nodes[nodes_explored - 1].y = neighbor->node->y;
                        
                        enqueue(open_set, neighbor->node);
                    }
                }
            }
            neighbor = neighbor->next;
        }
    }
    
    // Print path
    graph_node_t* current = graph[end_y][end_x];
    if (current != NULL) {
        // Count path length first
        int path_length = 0;
        graph_node_t* temp = current;
        while (temp != NULL) {
            path_length++;
            temp = temp->parent;
        }

        // Allocate memory for the path
        regular_result.path = (graph_node_t*)malloc(path_length * sizeof(graph_node_t));
        regular_result.path_length = path_length;
        regular_result.total_cost = current->g_cost;
        regular_result.nodes_explored = nodes_explored;

        // Print and store path points
        printf("\nPath from (%d,%d) to (%d,%d): ", start_x, start_y, end_x, end_y);
        int i = path_length - 1;
        temp = current;
        while (temp != NULL) {
            printf("(%d,%d) ", temp->x, temp->y);  // Print to terminal
            regular_result.path[i].x = temp->x;    // Store in result
            regular_result.path[i].y = temp->y;
            temp = temp->parent;
            i--;
        }
        printf("\nTotal path cost: %.2f\n", current->g_cost);
        printf("Total nodes explored: %d\n", nodes_explored);
    }
    
    free_priority_queue(open_set);
    free_priority_queue(closed_set);
    return regular_result;
}

// PID Controller update function
void update_pid_controller(PIDController* pid, double current_error, int x, int y) {
    double error = current_error;
    double derivative = error - pid->last_error;
    pid->integral += error;
    
    // Check if current position is in a slope region
    bool in_slope = false;
    for (int i = 0; i < num_slope_regions; i++) {
        if (x >= slope_regions[i][0] && x <= slope_regions[i][2] &&
            y >= slope_regions[i][1] && y <= slope_regions[i][3]) {
            in_slope = true;
            break;
        }
    }
    
    // Adjust error based on slope presence
    if (in_slope) {
        error *= 3.0;  // Double the error when in slope regions
        pid->integral *= 1.5;  // Increase integral term more aggressively
    }
    
    // Calculate weight adjustment with modified parameters
    double adjustment = pid->kp * error * 0.08    + 
                       pid->ki * pid->integral + 
                       pid->kd * derivative * 0.1;
    
    // Update weight with bounds
    pid->weight = adjustment;
    if (pid->weight < 0.1) pid->weight = 0.1;
    if (pid->weight > 10.0) pid->weight = 10.0;
    
    pid->last_error = error;
}

// Dynamic Weighted A* with PID controller
path_result_t dynamic_weighted_a_star(int start_x, int start_y, int end_x, int end_y) {
    path_result_t dynamic_result = {0};
    nodes_explored = 0;  // Reset counter at start
    
    // Initialize PID controller
    PIDController pid = {0};
    pid.kp = 0.8;
    pid.ki = 0.0;
    pid.kd = 0.01;
    pid.weight = 1.0;
    pid.last_error = heuristic(start_x, start_y, end_x, end_y);
    pid.integral = 0.0;
    
    PriorityQueue* open_set = create_priority_queue(1000);
    PriorityQueue* closed_set = create_priority_queue(1000);
    
    // Allocate memory for explored nodes (we'll resize if needed)
    dynamic_result.explored_nodes = (graph_node_t*)malloc(1000 * sizeof(graph_node_t));
    
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
    nodes_explored = 1;
    
    dynamic_result.explored_nodes[0].x = start_x;
    dynamic_result.explored_nodes[0].y = start_y;
    
    enqueue(open_set, graph[start_y][start_x]);
    
    printf("\nStarting PID A* search...\n");
    printf("Initial error: %.2f\n", graph[start_y][start_x]->h_cost);
    
    int iterations = 0;
    int max_iterations = 1000;
    
    while (!is_empty(open_set) && iterations < max_iterations) {
        iterations++;
        
        graph_node_t* current = dequeue(open_set);
        if (!current) break;
        
        // Update PID controller with current position
        double current_error = heuristic(current->x, current->y, end_x, end_y);
        update_pid_controller(&pid, current_error, current->x, current->y);
        
        if (current->x == end_x && current->y == end_y) {
            printf("Goal reached in %d iterations!\n", iterations);
            printf("Iteration %d: weight=%.2f, error=%.2f\n", 
                iterations, pid.weight, pid.last_error);
            break;
        }
        
        enqueue(closed_set, current);
        
        adj_list_node_t* neighbor = current->adj_list_head;
        while (neighbor) {
            if (!is_in_queue(closed_set, neighbor->node)) {
                double tentative_g = current->g_cost + neighbor->distance;
                
                // Check if neighbor is in slope region
                bool neighbor_in_slope = false;
                for (int i = 0; i < num_slope_regions; i++) {
                    if (neighbor->node->x >= slope_regions[i][0] && 
                        neighbor->node->x <= slope_regions[i][2] &&
                        neighbor->node->y >= slope_regions[i][1] && 
                        neighbor->node->y <= slope_regions[i][3]) {
                        neighbor_in_slope = true;
                        break;
                    }
                }
                
                // Apply additional cost for slope regions
                if (neighbor_in_slope) {
                    tentative_g *= 1.5;  // Increase cost by 50% in slope regions
                }
                
                if (tentative_g < neighbor->node->g_cost) {
                    neighbor->node->parent = current;
                    neighbor->node->g_cost = tentative_g;
                    neighbor->node->h_cost = heuristic(neighbor->node->x, neighbor->node->y, end_x, end_y);
                    neighbor->node->f_cost = neighbor->node->g_cost + pid.weight * neighbor->node->h_cost;
                    
                    if (!is_in_queue(open_set, neighbor->node)) {
                        nodes_explored++;
                        
                        // Store explored node
                        if (nodes_explored >= 1000) {
                            // Resize the array if needed
                            dynamic_result.explored_nodes = realloc(dynamic_result.explored_nodes, 
                                (nodes_explored + 1000) * sizeof(graph_node_t));
                        }
                        dynamic_result.explored_nodes[nodes_explored - 1].x = neighbor->node->x;
                        dynamic_result.explored_nodes[nodes_explored - 1].y = neighbor->node->y;
                        
                        enqueue(open_set, neighbor->node);
                    }
                }
            }
            neighbor = neighbor->next;
        }
        
        // Print PID values every 10 iterations
        if (iterations % 10 == 0) {
            printf("Iteration %d: weight=%.2f, error=%.2f\n", 
                   iterations, pid.weight, pid.last_error);
        }
    }
    
    if (iterations >= max_iterations) {
        printf("Maximum iterations reached!\n");
    }
    
    // When reconstructing path, do both printing and storing
    graph_node_t* current = graph[end_y][end_x];
    if (current != NULL) {
        // Count path length first
        int path_length = 0;
        graph_node_t* temp = current;
        while (temp != NULL) {
            path_length++;
            temp = temp->parent;
        }

        // Allocate memory for the path
        dynamic_result.path = (graph_node_t*)malloc(path_length * sizeof(graph_node_t));
        dynamic_result.path_length = path_length;
        dynamic_result.total_cost = current->g_cost;
        dynamic_result.nodes_explored = nodes_explored;

        // Print and store path points
        printf("\nFinal path from (%d,%d) to (%d,%d):\n", start_x, start_y, end_x, end_y);
        int i = path_length - 1;
        temp = current;
        while (temp != NULL) {
            printf("(%d,%d) ", temp->x, temp->y);  // Print to terminal
            dynamic_result.path[i].x = temp->x;    // Store in result
            dynamic_result.path[i].y = temp->y;
            temp = temp->parent;
            i--;
        }
        printf("\nTotal path cost: %.2f\n", current->g_cost);
        printf("Total nodes explored: %d\n", nodes_explored);
    }
    
    free_priority_queue(open_set);
    free_priority_queue(closed_set);
    return dynamic_result;
}

// Bidirectional A* algorithm with safety checks
path_result_t bidirectional_a_star(int start_x, int start_y, int end_x, int end_y) {
    path_result_t bidirectional_result = {0};
    nodes_explored = 0;  // Reset counter at start
    
    printf("Starting bidirectional A* from (%d,%d) to (%d,%d)\n", start_x, start_y, end_x, end_y);
    
    init_queue_fwd();
    init_queue_bwd();

    // Allocate memory for explored nodes (we'll resize if needed)
    bidirectional_result.explored_nodes = (graph_node_t*)malloc(1000 * sizeof(graph_node_t));
    
    // Reset graph
    for (int y = 0; y < GRID_SIZE; y++) {
        for (int x = 0; x < GRID_SIZE; x++) {
            graph[y][x]->g_cost = MAX_COST;
            graph[y][x]->g_cost_bwd = MAX_COST;
            graph[y][x]->h_cost = 0.0;
            graph[y][x]->f_cost = MAX_COST;
            graph[y][x]->parent = NULL;
            graph[y][x]->backward_parent = NULL;
            graph[y][x]->reached = 0;
        }
    }
    
    // Check if start and end nodes are valid
    if (start_x < 0 || start_x >= GRID_SIZE || start_y < 0 || start_y >= GRID_SIZE ||
        end_x < 0 || end_x >= GRID_SIZE || end_y < 0 || end_y >= GRID_SIZE) {
        printf("Invalid start or end coordinates!\n");
        return bidirectional_result;
    }

    graph[start_y][start_x]->g_cost = 0;
    graph[start_y][start_x]->h_cost = heuristic(start_x, start_y, end_x, end_y);
    graph[start_y][start_x]->f_cost = graph[start_y][start_x]->h_cost;
    graph[start_y][start_x]->reached = 1;
    nodes_explored = 2;  // Count both start and end nodes
    
    // Store start node in explored nodes
    bidirectional_result.explored_nodes[0].x = start_x;
    bidirectional_result.explored_nodes[0].y = start_y;
    
    // Set end node
    graph[end_y][end_x]->g_cost_bwd = 0;
    graph[end_y][end_x]->h_cost = heuristic(end_x, end_y, start_x, start_y);
    graph[end_y][end_x]->f_cost = graph[end_y][end_x]->h_cost;
    graph[end_y][end_x]->reached = 2;
    
    // Store end node in explored nodes
    bidirectional_result.explored_nodes[1].x = end_x;
    bidirectional_result.explored_nodes[1].y = end_y;
    
    insert_fwd(graph[start_y][start_x], graph[start_y][start_x]->f_cost, &queue_fwd, &queue_size_fwd, &max_queue_size_fwd);
    insert_bwd(graph[end_y][end_x], graph[end_y][end_x]->f_cost, &queue_bwd, &queue_size_bwd, &max_queue_size_bwd);

    graph_node_t* intersection = NULL;
    double best_path_cost = MAX_COST;

    while (!is_empty_fwd() && !is_empty_bwd() && !intersection) {
        // Forward search
        graph_node_t* current_fwd = get_fwd(queue_fwd, &queue_size_fwd);
        if (!current_fwd) break;
        
        adj_list_node_t* neighbor = current_fwd->adj_list_head;
        while (neighbor && neighbor->node) {
            double tentative_g = current_fwd->g_cost + neighbor->distance;
            if (tentative_g < neighbor->node->g_cost) {
                neighbor->node->parent = current_fwd;
                neighbor->node->g_cost = tentative_g;
                neighbor->node->h_cost = heuristic(neighbor->node->x, neighbor->node->y, end_x, end_y);
                neighbor->node->f_cost = tentative_g + neighbor->node->h_cost;
                
                if (neighbor->node->reached == 2) {
                    double total_cost = tentative_g + neighbor->node->g_cost_bwd;
                    if (total_cost < best_path_cost) {
                        best_path_cost = total_cost;
                        intersection = neighbor->node;
                        break;  // Found intersection, stop forward search
                    }
                } else if (neighbor->node->reached == 0) {
                    neighbor->node->reached = 1;
                    nodes_explored++;  // Only count when first visited
                    
                    // Store explored node
                    if (nodes_explored >= 1000) {
                        // Resize the array if needed
                        bidirectional_result.explored_nodes = realloc(bidirectional_result.explored_nodes, 
                            (nodes_explored + 1000) * sizeof(graph_node_t));
                    }
                    bidirectional_result.explored_nodes[nodes_explored - 1].x = neighbor->node->x;
                    bidirectional_result.explored_nodes[nodes_explored - 1].y = neighbor->node->y;
                    
                    insert_fwd(neighbor->node, neighbor->node->f_cost, &queue_fwd, &queue_size_fwd, &max_queue_size_fwd);
                }
            }
            neighbor = neighbor->next;
        }
        if (intersection) break;  // Stop if intersection found
        
        // Backward search
        graph_node_t* current_bwd = get_bwd(queue_bwd, &queue_size_bwd);
        if (!current_bwd) break;
        
        neighbor = current_bwd->adj_list_head;
        while (neighbor && neighbor->node) {
            double tentative_g = current_bwd->g_cost_bwd + neighbor->distance;
            if (tentative_g < neighbor->node->g_cost_bwd) {
                neighbor->node->backward_parent = current_bwd;
                neighbor->node->g_cost_bwd = tentative_g;
                neighbor->node->h_cost = heuristic(neighbor->node->x, neighbor->node->y, start_x, start_y);
                neighbor->node->f_cost = tentative_g + neighbor->node->h_cost;
                
                if (neighbor->node->reached == 1) {
                    double total_cost = tentative_g + neighbor->node->g_cost;
                    if (total_cost < best_path_cost) {
                        best_path_cost = total_cost;
                        intersection = neighbor->node;
                        break;  // Found intersection, stop backward search
                    }
                } else if (neighbor->node->reached == 0) {
                    neighbor->node->reached = 2;
                    nodes_explored++;  // Only count when first visited
                    
                    // Store explored node
                    if (nodes_explored >= 1000) {
                        // Resize the array if needed
                        bidirectional_result.explored_nodes = realloc(bidirectional_result.explored_nodes, 
                            (nodes_explored + 1000) * sizeof(graph_node_t));
                    }
                    bidirectional_result.explored_nodes[nodes_explored - 1].x = neighbor->node->x;
                    bidirectional_result.explored_nodes[nodes_explored - 1].y = neighbor->node->y;
                    
                    insert_bwd(neighbor->node, neighbor->node->f_cost, &queue_bwd, &queue_size_bwd, &max_queue_size_bwd);
                }
            }
            neighbor = neighbor->next;
        }
        if (intersection) break;  // Stop if intersection found
    }
    
    printf("Search completed\n");
    if (intersection) {
        printf("\nIntersection found at (%d,%d)\n", intersection->x, intersection->y);
        
        // Count forward path length (from intersection to start)
        int forward_length = 0;
        graph_node_t* temp = intersection;
        while (temp != NULL) {
            forward_length++;
            temp = temp->parent;
        }

        // Count backward path length (from intersection to end)
        int backward_length = 0;
        temp = intersection;
        while (temp != NULL) {
            backward_length++;
            temp = temp->backward_parent;
        }

        // Allocate memory for the complete path
        bidirectional_result.path = (graph_node_t*)malloc((forward_length + backward_length - 1) * sizeof(graph_node_t));
        bidirectional_result.path_length = forward_length + backward_length - 1;
        bidirectional_result.total_cost = best_path_cost;
        bidirectional_result.nodes_explored = nodes_explored;

        // First, store the forward path (from start to intersection)
        printf("Forward path (start to intersection): ");
        int i = forward_length - 1;
        temp = intersection;
        while (temp != NULL) {
            printf("(%d,%d) ", temp->x, temp->y);
            bidirectional_result.path[i].x = temp->x;
            bidirectional_result.path[i].y = temp->y;
            temp = temp->parent;
            i--;
        }
        printf("\n");

        // Then, store the backward path (from intersection to end)
        // Skip the intersection point to avoid duplication
        printf("Backward path (intersection to goal): ");
        temp = intersection->backward_parent;
        i = forward_length;
        while (temp != NULL) {
            printf("(%d,%d) ", temp->x, temp->y);
            bidirectional_result.path[i].x = temp->x;
            bidirectional_result.path[i].y = temp->y;
            temp = temp->backward_parent;
            i++;
        }
        printf("\n");

        // Verify the path is continuous
        printf("Verifying path continuity...\n");
        bool path_valid = true;
        for (int i = 0; i < bidirectional_result.path_length - 1; i++) {
            int dx = abs(bidirectional_result.path[i+1].x - bidirectional_result.path[i].x);
            int dy = abs(bidirectional_result.path[i+1].y - bidirectional_result.path[i].y);
            if (dx > 1 || dy > 1) {
                printf("Warning: Path discontinuity between (%d,%d) and (%d,%d)\n",
                       bidirectional_result.path[i].x, bidirectional_result.path[i].y,
                       bidirectional_result.path[i+1].x, bidirectional_result.path[i+1].y);
                path_valid = false;
            }
        }
        if (!path_valid) {
            printf("Path contains invalid moves!\n");
        }

        printf("Total path cost: %.2f\n", best_path_cost);
        printf("Total nodes explored: %d\n", nodes_explored);
    } else {
        printf("No path found between (%d,%d) and (%d,%d)!\n", start_x, start_y, end_x, end_y);
    }
    
    free_queue_fwd();
    free_queue_bwd();
    return bidirectional_result;
}

// Main function
// In main, replace the JSON writing sections with:
int main() {
    init_graph();
    
    // Test with simple grid
    printf("=== Testing on Simple Grid ===\n");
    setup_simple_grid();
    
    printf("\nSimple Grid - Regular A* from (0,0) to (%d,%d)\n", GRID_SIZE - 1, GRID_SIZE - 1);
    path_result_t regular_result = a_star(0, 0, GRID_SIZE - 1, GRID_SIZE - 1);
    
    printf("\nSimple Grid - Bidirectional A* from (0,0) to (%d,%d)\n", GRID_SIZE - 1, GRID_SIZE - 1);
    path_result_t bidirectional_result = bidirectional_a_star(0, 0, GRID_SIZE - 1, GRID_SIZE - 1);
    
    printf("\nSimple Grid - PID A* from (0,0) to (%d,%d)\n", GRID_SIZE - 1, GRID_SIZE - 1);
    path_result_t dynamic_result = dynamic_weighted_a_star(0, 0, GRID_SIZE - 1, GRID_SIZE - 1);
    
    // Write simple grid results
    FILE* json_file_simple = fopen("result_simple.json", "w");
    if (json_file_simple) {
        fprintf(json_file_simple, "{\n");
        
        // Grid setup
        fprintf(json_file_simple, "  \"grid_setup\": {\n");
        fprintf(json_file_simple, "    \"size\": %d,\n", GRID_SIZE);
        fprintf(json_file_simple, "    \"obstacles\": [],\n");
        fprintf(json_file_simple, "    \"slope_regions\": []\n");
        fprintf(json_file_simple, "  },\n");
        
        // Regular A* results
        fprintf(json_file_simple, "  \"regular_a_star\": {\n");
        fprintf(json_file_simple, "    \"path\": [\n");
        for (int i = 0; i < regular_result.path_length; i++) {
            fprintf(json_file_simple, "      {\"x\": %d, \"y\": %d}%s\n", 
                    regular_result.path[i].x, regular_result.path[i].y,
                    i < regular_result.path_length - 1 ? "," : "");
        }
        fprintf(json_file_simple, "    ],\n");
        fprintf(json_file_simple, "    \"explored_nodes\": [\n");
        for (int i = 0; i < regular_result.nodes_explored; i++) {
            fprintf(json_file_simple, "      {\"x\": %d, \"y\": %d}%s\n", 
                    regular_result.explored_nodes[i].x, regular_result.explored_nodes[i].y,
                    i < regular_result.nodes_explored - 1 ? "," : "");
        }
        fprintf(json_file_simple, "    ],\n");
        fprintf(json_file_simple, "    \"total_cost\": %.2f,\n", regular_result.total_cost);
        fprintf(json_file_simple, "    \"nodes_explored\": %d\n", regular_result.nodes_explored);
        fprintf(json_file_simple, "  },\n");
        
        // Bidirectional A* results
        fprintf(json_file_simple, "  \"bidirectional_a_star\": {\n");
        fprintf(json_file_simple, "    \"path\": [\n");
        for (int i = 0; i < bidirectional_result.path_length; i++) {
            fprintf(json_file_simple, "      {\"x\": %d, \"y\": %d}%s\n", 
                    bidirectional_result.path[i].x, bidirectional_result.path[i].y,
                    i < bidirectional_result.path_length - 1 ? "," : "");
        }
        fprintf(json_file_simple, "    ],\n");
        fprintf(json_file_simple, "    \"explored_nodes\": [\n");
        for (int i = 0; i < bidirectional_result.nodes_explored; i++) {
            fprintf(json_file_simple, "      {\"x\": %d, \"y\": %d}%s\n", 
                    bidirectional_result.explored_nodes[i].x, bidirectional_result.explored_nodes[i].y,
                    i < bidirectional_result.nodes_explored - 1 ? "," : "");
        }
        fprintf(json_file_simple, "    ],\n");
        fprintf(json_file_simple, "    \"total_cost\": %.2f,\n", bidirectional_result.total_cost);
        fprintf(json_file_simple, "    \"nodes_explored\": %d\n", bidirectional_result.nodes_explored);
        fprintf(json_file_simple, "  },\n");
        
        // Dynamic A* results
        fprintf(json_file_simple, "  \"dynamic_a_star\": {\n");
        fprintf(json_file_simple, "    \"path\": [\n");
        for (int i = 0; i < dynamic_result.path_length; i++) {
            fprintf(json_file_simple, "      {\"x\": %d, \"y\": %d}%s\n", 
                    dynamic_result.path[i].x, dynamic_result.path[i].y,
                    i < dynamic_result.path_length - 1 ? "," : "");
        }
        fprintf(json_file_simple, "    ],\n");
        fprintf(json_file_simple, "    \"explored_nodes\": [\n");
        for (int i = 0; i < dynamic_result.nodes_explored; i++) {
            fprintf(json_file_simple, "      {\"x\": %d, \"y\": %d}%s\n", 
                    dynamic_result.explored_nodes[i].x, dynamic_result.explored_nodes[i].y,
                    i < dynamic_result.nodes_explored - 1 ? "," : "");
        }
        fprintf(json_file_simple, "    ],\n");
        fprintf(json_file_simple, "    \"total_cost\": %.2f,\n", dynamic_result.total_cost);
        fprintf(json_file_simple, "    \"nodes_explored\": %d\n", dynamic_result.nodes_explored);
        fprintf(json_file_simple, "  }\n");
        fprintf(json_file_simple, "}\n");
        
        fclose(json_file_simple);
        printf("Simple grid results written to result_simple.json\n");
    } else {
        printf("Error opening result_simple.json for writing\n");
    }
    
    // Test with complicated grid
    printf("\n=== Testing on Complicated Grid ===\n");
    setup_complicated_grid();
    
    printf("\nComplicated Grid - Regular A* from (0,0) to (%d,%d)\n", GRID_SIZE - 1, GRID_SIZE - 1);
    path_result_t regular_complex_result = a_star(0, 0, GRID_SIZE - 1, GRID_SIZE - 1);
    
    printf("\nComplicated Grid - Bidirectional A* from (0,0) to (%d,%d)\n", GRID_SIZE - 1, GRID_SIZE - 1);
    path_result_t bidirectional_complex_result = bidirectional_a_star(0, 0, GRID_SIZE - 1, GRID_SIZE - 1);
    
    printf("\nComplicated Grid - PID A* from (0,0) to (%d,%d)\n", GRID_SIZE - 1, GRID_SIZE - 1);
    path_result_t dynamic_complex_result = dynamic_weighted_a_star(0, 0, GRID_SIZE - 1, GRID_SIZE - 1);
    
    // Write complicated grid results
    FILE* json_file_complicated = fopen("result_complicated.json", "w");
    if (json_file_complicated) {
        fprintf(json_file_complicated, "{\n");
        
        // Grid setup
        fprintf(json_file_complicated, "  \"grid_setup\": {\n");
        fprintf(json_file_complicated, "    \"size\": %d,\n", GRID_SIZE);
        fprintf(json_file_complicated, "    \"obstacles\": [\n");
        for (int i = 0; i < obstacle_count; i++) {
            fprintf(json_file_complicated, "      [%d, %d]%s\n", 
                    obstacles[i][0], obstacles[i][1],
                    i < obstacle_count - 1 ? "," : "");
        }
        fprintf(json_file_complicated, "    ],\n");
        fprintf(json_file_complicated, "    \"slope_regions\": [\n");
        for (int i = 0; i < num_slope_regions; i++) {
            fprintf(json_file_complicated, "      [%d, %d, %d, %d]%s\n", 
                    slope_regions[i][0], slope_regions[i][1],
                    slope_regions[i][2], slope_regions[i][3],
                    i < num_slope_regions - 1 ? "," : "");
        }
        fprintf(json_file_complicated, "    ]\n");
        fprintf(json_file_complicated, "  },\n");
        
        // Regular A* results
        fprintf(json_file_complicated, "  \"regular_a_star\": {\n");
        fprintf(json_file_complicated, "    \"path\": [\n");
        for (int i = 0; i < regular_complex_result.path_length; i++) {
            fprintf(json_file_complicated, "      {\"x\": %d, \"y\": %d}%s\n", 
                    regular_complex_result.path[i].x, regular_complex_result.path[i].y,
                    i < regular_complex_result.path_length - 1 ? "," : "");
        }
        fprintf(json_file_complicated, "    ],\n");
        fprintf(json_file_complicated, "    \"explored_nodes\": [\n");
        for (int i = 0; i < regular_complex_result.nodes_explored; i++) {
            fprintf(json_file_complicated, "      {\"x\": %d, \"y\": %d}%s\n", 
                    regular_complex_result.explored_nodes[i].x, regular_complex_result.explored_nodes[i].y,
                    i < regular_complex_result.nodes_explored - 1 ? "," : "");
        }
        fprintf(json_file_complicated, "    ],\n");
        fprintf(json_file_complicated, "    \"total_cost\": %.2f,\n", regular_complex_result.total_cost);
        fprintf(json_file_complicated, "    \"nodes_explored\": %d\n", regular_complex_result.nodes_explored);
        fprintf(json_file_complicated, "  },\n");
        
        // Bidirectional A* results
        fprintf(json_file_complicated, "  \"bidirectional_a_star\": {\n");
        fprintf(json_file_complicated, "    \"path\": [\n");
        for (int i = 0; i < bidirectional_complex_result.path_length; i++) {
            fprintf(json_file_complicated, "      {\"x\": %d, \"y\": %d}%s\n", 
                    bidirectional_complex_result.path[i].x, bidirectional_complex_result.path[i].y,
                    i < bidirectional_complex_result.path_length - 1 ? "," : "");
        }
        fprintf(json_file_complicated, "    ],\n");
        fprintf(json_file_complicated, "    \"explored_nodes\": [\n");
        for (int i = 0; i < bidirectional_complex_result.nodes_explored; i++) {
            fprintf(json_file_complicated, "      {\"x\": %d, \"y\": %d}%s\n", 
                    bidirectional_complex_result.explored_nodes[i].x, bidirectional_complex_result.explored_nodes[i].y,
                    i < bidirectional_complex_result.nodes_explored - 1 ? "," : "");
        }
        fprintf(json_file_complicated, "    ],\n");
        fprintf(json_file_complicated, "    \"total_cost\": %.2f,\n", bidirectional_complex_result.total_cost);
        fprintf(json_file_complicated, "    \"nodes_explored\": %d\n", bidirectional_complex_result.nodes_explored);
        fprintf(json_file_complicated, "  },\n");
        
        // Dynamic A* results
        fprintf(json_file_complicated, "  \"dynamic_a_star\": {\n");
        fprintf(json_file_complicated, "    \"path\": [\n");
        for (int i = 0; i < dynamic_complex_result.path_length; i++) {
            fprintf(json_file_complicated, "      {\"x\": %d, \"y\": %d}%s\n", 
                    dynamic_complex_result.path[i].x, dynamic_complex_result.path[i].y,
                    i < dynamic_complex_result.path_length - 1 ? "," : "");
        }
        fprintf(json_file_complicated, "    ],\n");
        fprintf(json_file_complicated, "    \"explored_nodes\": [\n");
        for (int i = 0; i < dynamic_complex_result.nodes_explored; i++) {
            fprintf(json_file_complicated, "      {\"x\": %d, \"y\": %d}%s\n", 
                    dynamic_complex_result.explored_nodes[i].x, dynamic_complex_result.explored_nodes[i].y,
                    i < dynamic_complex_result.nodes_explored - 1 ? "," : "");
        }
        fprintf(json_file_complicated, "    ],\n");
        fprintf(json_file_complicated, "    \"total_cost\": %.2f,\n", dynamic_complex_result.total_cost);
        fprintf(json_file_complicated, "    \"nodes_explored\": %d\n", dynamic_complex_result.nodes_explored);
        fprintf(json_file_complicated, "  }\n");
        fprintf(json_file_complicated, "}\n");
        
        fclose(json_file_complicated);
        printf("Complicated grid results written to result_complicated.json\n");
    } else {
        printf("Error opening result_complicated.json for writing\n");
    }
    
    // Free allocated memory
    free(regular_result.path);
    free(regular_result.explored_nodes);
    free(bidirectional_result.path);
    free(bidirectional_result.explored_nodes);
    free(dynamic_result.path);
    free(dynamic_result.explored_nodes);
    free(regular_complex_result.path);
    free(regular_complex_result.explored_nodes);
    free(bidirectional_complex_result.path);
    free(bidirectional_complex_result.explored_nodes);
    free(dynamic_complex_result.path);
    free(dynamic_complex_result.explored_nodes);
    
    return 0;
}






