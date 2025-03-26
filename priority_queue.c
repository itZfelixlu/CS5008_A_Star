/**
 * @file priority_queue.c
 * @brief Binary heap-based priority queue implementation for the A* algorithm.
 * TODO: Add details.
 *
 * @author YOUR NAME AND UNIVERSITY EMAIL/ID
 */

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#include "priority_queue.h"
#include "a_star.h"

size_t queue_size = 0;
size_t max_queue_size = INIT_QUEUE_SIZE;
queue_node_t** queue = NULL;

// Bidirectional A*
size_t queue_size_fwd = 0;
size_t max_queue_size_fwd = INIT_QUEUE_SIZE;
queue_node_t** queue_fwd = NULL;

size_t queue_size_bwd = 0;
size_t max_queue_size_bwd = INIT_QUEUE_SIZE;
queue_node_t** queue_bwd = NULL;

void init_queue() {
    queue = malloc(sizeof(queue_node_t*) * INIT_QUEUE_SIZE);
}

void resize_queue() {
    if (max_queue_size * 2 < SIZE_MAX) {
        max_queue_size *= 2;
        queue = realloc(queue, sizeof(queue_node_t*) * max_queue_size);
    } else {
        fprintf(stderr, "Priority queue has reached maximum possible size.\n");
        for (int i = 0; i < queue_size; i++) {
            free(*(queue + i));
        }
        free(queue);
        // TODO: FREE GRAPH+ADJ_LIST NODES?
        exit(1);
    }
}

// void swap(size_t i, size_t j) {
//     if (i < 0 || j < 0 || i >= queue_size || j >= queue_size) {
//         fprintf(stderr, "Swap indices out of bounds.\n");
//     } else {
//         queue_node_t* temp = *(queue + i);
//         *(queue + i) = *(queue + j);
//         *(queue + j) = temp;
//     }
// }

// Modified swap function to take in which queue (bwd or fwd) as argument
void swap(size_t i, size_t j, queue_node_t** q, size_t q_size) {
    if (i < 0 || j < 0 || i >= q_size || j >= q_size) {
        fprintf(stderr, "Swap indices out of bounds.\n");
    } else {
        queue_node_t* temp = q[i];
        q[i] = q[j];
        q[j] = temp;
    }
}

// void heapify(size_t i) {
//     size_t highest_priority = i;
//     size_t left = 2 * i + 1;
//     if (left < queue_size && (*(queue + left))->priority < (*(queue + highest_priority))->priority) {
//         highest_priority = left;
//     }
//     size_t right = 2 * i + 2;
//     if (right < queue_size && (*(queue + right))->priority < (*(queue + highest_priority))->priority) {
//         highest_priority = right;
//     }
//     if (i != highest_priority) {
//         swap(i, highest_priority);
//         heapify(highest_priority);
//     }
// }

// Modified heapity function to also apply for forward and backward queue
// Rename varaible highest_priority to smallest (min-heap)
// smaller -> more important
void heapify(size_t i, queue_node_t** q, size_t q_size) {
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

void insert(graph_node_t* node, int priority) {
    queue_size++;
    if (queue_size >= max_queue_size) resize_queue();
    queue_node_t* new_node = malloc(sizeof(queue_node_t));
    new_node->node = node;
    new_node->priority = priority;
    *(queue + queue_size - 1) = new_node;
    size_t i = queue_size - 1;
    size_t parent = (i - 1) / 2;
    while (i > 0 && parent >= 0 && (*(queue + parent))->priority > (*(queue + i))->priority) {
        swap(parent, i, queue, queue_size);
        i = parent;
        parent = (i - 1) / 2;
    }
}

graph_node_t* get() {
    if (queue_size == 0)
        return NULL;
    queue_size--; // Before decrement, last node is at queue[queue_size - 1]
    graph_node_t* result = (*queue)->node;
    swap(0, queue_size, queue, queue_size + 1); //Decrement first makes the last node is at queue_size index
    free(*(queue + queue_size));
    heapify(0, queue, queue_size);
    return result;
}

int is_empty() {
    return (queue_size == 0);
}

void free_queue() {
    for (int i = 0; i < queue_size; i++) {
        free(*(queue + i));
    }
    free(queue);
    // TODO: FREE GRAPH NODES?
}

// Duplicate functions for forward queue
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

// Duplicate functions for backward queue
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
