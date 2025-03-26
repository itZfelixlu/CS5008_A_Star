/**
 * @file priority_queue.h
 * @brief Binary heap-based priority queue header file for the A* algorithm.
 * TODO: Add details.
 *
 * @author YOUR NAME AND UNIVERSITY EMAIL/ID
 */

#include "a_star.h"

#define INIT_QUEUE_SIZE 25

// Original queue (unidirectional A*)
extern size_t queue_size;
extern size_t max_queue_size;
extern queue_node_t** queue;

// Forward queue (bidirectional)
extern size_t queue_size_fwd;
extern size_t max_queue_size_fwd;
extern queue_node_t** queue_fwd;

// Backward queue (bidirectional)
extern size_t queue_size_bwd;
extern size_t max_queue_size_bwd;
extern queue_node_t** queue_bwd;

typedef struct queue_node {
    struct graph_node* node;
    int priority;
} queue_node_t;

// Original functions
void init_queue();
void insert(graph_node_t* node, int priority);
graph_node_t* get();
int is_empty();
void free_queue();

// Forward functions
void init_queue_fwd();
void insert_fwd(graph_node_t* node, int priority, queue_node_t*** q, size_t* q_size, size_t* max_q_size);
graph_node_t* get_fwd(queue_node_t** q, size_t* q_size);
int is_empty_fwd();
void free_queue_fwd();

// Backward functions
void init_queue_bwd();
void insert_bwd(graph_node_t* node, int priority, queue_node_t*** q, size_t* q_size, size_t* max_q_size);
graph_node_t* get_bwd(queue_node_t** q, size_t* q_size);
int is_empty_bwd();
void free_queue_bwd();
