/**
 * @file priority_queue.h
 * @brief Binary heap-based priority queue header file for the A* algorithm.
 * TODO: Add details.
 *
 * @author YOUR NAME AND UNIVERSITY EMAIL/ID
 */

#ifndef PRIORITY_QUEUE_H
#define PRIORITY_QUEUE_H

#include <stddef.h>
#include "a_star.h"

#define INIT_QUEUE_SIZE 1000

// Queue node structure for bidirectional A*
typedef struct queue_node {
    graph_node_t* node;
    double priority;
} queue_node_t;

// External variables for bidirectional A*
extern size_t queue_size_fwd;
extern size_t max_queue_size_fwd;
extern queue_node_t** queue_fwd;

extern size_t queue_size_bwd;
extern size_t max_queue_size_bwd;
extern queue_node_t** queue_bwd;

// Priority queue structure
typedef struct {
    graph_node_t** nodes;
    size_t size;
    size_t capacity;
} PriorityQueue;

// Function declarations for bidirectional A*
void init_queue_fwd();
void insert_fwd(graph_node_t* node, double priority, queue_node_t*** q, size_t* q_size, size_t* max_q_size);
graph_node_t* get_fwd(queue_node_t** q, size_t* q_size);
int is_empty_fwd();
void free_queue_fwd();

void init_queue_bwd();
void insert_bwd(graph_node_t* node, double priority, queue_node_t*** q, size_t* q_size, size_t* max_q_size);
graph_node_t* get_bwd(queue_node_t** q, size_t* q_size);
int is_empty_bwd();
void free_queue_bwd();

// Function declarations for priority queue
PriorityQueue* create_priority_queue(size_t capacity);
void free_priority_queue(PriorityQueue* queue);
void enqueue(PriorityQueue* queue, graph_node_t* node);
graph_node_t* dequeue(PriorityQueue* queue);
int is_empty(PriorityQueue* queue);
int is_in_queue(PriorityQueue* queue, graph_node_t* node);

#endif // PRIORITY_QUEUE_H
