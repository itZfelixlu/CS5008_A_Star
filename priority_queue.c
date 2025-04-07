/**
 * @file priority_queue.c
 * @brief Binary heap-based priority queue implementation for the A* algorithm.
 *
 * @author YOUR NAME AND UNIVERSITY EMAIL/ID
 */

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#include "priority_queue.h"
#include "a_star.h"

// Bidirectional A* queue variables
size_t queue_size_fwd = 0;
size_t max_queue_size_fwd = INIT_QUEUE_SIZE;
queue_node_t** queue_fwd = NULL;

size_t queue_size_bwd = 0;
size_t max_queue_size_bwd = INIT_QUEUE_SIZE;
queue_node_t** queue_bwd = NULL;

// Helper functions for bidirectional A*
static void swap(size_t i, size_t j, queue_node_t** q, size_t q_size) {
    if (i < 0 || j < 0 || i >= q_size || j >= q_size) {
        fprintf(stderr, "Swap indices out of bounds.\n");
    } else {
        queue_node_t* temp = q[i];
        q[i] = q[j];
        q[j] = temp;
    }
}

static void heapify(size_t i, queue_node_t** q, size_t q_size) {
    size_t smallest = i;
    size_t left = 2 * i + 1;
    size_t right = 2 * i + 2;
    if (left < q_size && q[left]->priority < q[smallest]->priority) {
        smallest = left;
    }
    if (right < q_size && q[right]->priority < q[smallest]->priority) {
        smallest = right;
    }
    if (smallest != i) {
        swap(i, smallest, q, q_size);
        heapify(smallest, q, q_size);
    }
}

// Forward queue functions
void init_queue_fwd() {
    queue_fwd = malloc(sizeof(queue_node_t*) * INIT_QUEUE_SIZE);
}

void insert_fwd(graph_node_t* node, int priority, queue_node_t*** q, size_t* q_size, size_t* max_q_size) {
    (*q_size)++;
    if (*q_size >= *max_q_size) {
        *max_q_size *= 2;
        *q = realloc(*q, sizeof(queue_node_t*) * (*max_q_size));
    }
    queue_node_t* new_node = malloc(sizeof(queue_node_t));
    new_node->node = node;
    new_node->priority = priority;
    (*q)[*q_size - 1] = new_node;
    size_t i = *q_size - 1;
    size_t parent = (i - 1) / 2;
    while (i > 0 && (*q)[parent]->priority > (*q)[i]->priority) {
        swap(parent, i, *q, *q_size);
        i = parent;
        parent = (i - 1) / 2;
    }
}

graph_node_t* get_fwd(queue_node_t** q, size_t* q_size) {
    if (*q_size == 0) return NULL;
    graph_node_t* result = q[0]->node;
    (*q_size)--;
    swap(0, *q_size, q, *q_size + 1);
    free(q[*q_size]);
    heapify(0, q, *q_size);
    return result;
}

int is_empty_fwd() {
    return (queue_size_fwd == 0);
}

void free_queue_fwd() {
    for (int i = 0; i < queue_size_fwd; i++) {
        free(queue_fwd[i]);
    }
    free(queue_fwd);
}

// Backward queue functions
void init_queue_bwd() {
    queue_bwd = malloc(sizeof(queue_node_t*) * INIT_QUEUE_SIZE);
}

void insert_bwd(graph_node_t* node, int priority, queue_node_t*** q, size_t* q_size, size_t* max_q_size) {
    (*q_size)++;
    if (*q_size >= *max_q_size) {
        *max_q_size *= 2;
        *q = realloc(*q, sizeof(queue_node_t*) * (*max_q_size));
    }
    queue_node_t* new_node = malloc(sizeof(queue_node_t));
    new_node->node = node;
    new_node->priority = priority;
    (*q)[*q_size - 1] = new_node;
    size_t i = *q_size - 1;
    size_t parent = (i - 1) / 2;
    while (i > 0 && (*q)[parent]->priority > (*q)[i]->priority) {
        swap(parent, i, *q, *q_size);
        i = parent;
        parent = (i - 1) / 2;
    }
}

graph_node_t* get_bwd(queue_node_t** q, size_t* q_size) {
    if (*q_size == 0) return NULL;
    graph_node_t* result = q[0]->node;
    (*q_size)--;
    swap(0, *q_size, q, *q_size + 1);
    free(q[*q_size]);
    heapify(0, q, *q_size);
    return result;
}

int is_empty_bwd() {
    return (queue_size_bwd == 0);
}

void free_queue_bwd() {
    for (int i = 0; i < queue_size_bwd; i++) {
        free(queue_bwd[i]);
    }
    free(queue_bwd);
}

// New priority queue implementation
static void swap_nodes(graph_node_t** a, graph_node_t** b) {
    graph_node_t* temp = *a;
    *a = *b;
    *b = temp;
}

static void heapify_up(PriorityQueue* queue, size_t index) {
    while (index > 0) {
        size_t parent = (index - 1) / 2;
        if (queue->nodes[index]->f_cost < queue->nodes[parent]->f_cost) {
            swap_nodes(&queue->nodes[index], &queue->nodes[parent]);
            index = parent;
        } else {
            break;
        }
    }
}

static void heapify_down(PriorityQueue* queue, size_t index) {
    size_t left, right, smallest;
    
    while (1) {
        left = 2 * index + 1;
        right = 2 * index + 2;
        smallest = index;
        
        if (left < queue->size && 
            queue->nodes[left]->f_cost < queue->nodes[smallest]->f_cost) {
            smallest = left;
        }
        
        if (right < queue->size && 
            queue->nodes[right]->f_cost < queue->nodes[smallest]->f_cost) {
            smallest = right;
        }
        
        if (smallest != index) {
            swap_nodes(&queue->nodes[index], &queue->nodes[smallest]);
            index = smallest;
        } else {
            break;
        }
    }
}

PriorityQueue* create_priority_queue(size_t capacity) {
    PriorityQueue* queue = (PriorityQueue*)malloc(sizeof(PriorityQueue));
    if (!queue) return NULL;
    
    queue->nodes = (graph_node_t**)malloc(capacity * sizeof(graph_node_t*));
    if (!queue->nodes) {
        free(queue);
        return NULL;
    }
    
    queue->size = 0;
    queue->capacity = capacity;
    return queue;
}

void free_priority_queue(PriorityQueue* queue) {
    if (queue) {
        free(queue->nodes);
        free(queue);
    }
}

void enqueue(PriorityQueue* queue, graph_node_t* node) {
    if (queue->size >= queue->capacity) {
        // Resize the queue if needed
        size_t new_capacity = queue->capacity * 2;
        graph_node_t** new_nodes = (graph_node_t**)realloc(queue->nodes, 
                                                          new_capacity * sizeof(graph_node_t*));
        if (!new_nodes) return;
        queue->nodes = new_nodes;
        queue->capacity = new_capacity;
    }
    
    queue->nodes[queue->size] = node;
    heapify_up(queue, queue->size);
    queue->size++;
}

graph_node_t* dequeue(PriorityQueue* queue) {
    if (queue->size == 0) return NULL;
    
    graph_node_t* result = queue->nodes[0];
    queue->size--;
    
    if (queue->size > 0) {
        queue->nodes[0] = queue->nodes[queue->size];
        heapify_down(queue, 0);
    }
    
    return result;
}

int is_empty(PriorityQueue* queue) {
    return queue->size == 0;
}

int is_in_queue(PriorityQueue* queue, graph_node_t* node) {
    for (size_t i = 0; i < queue->size; i++) {
        if (queue->nodes[i] == node) return 1;
    }
    return 0;
}
