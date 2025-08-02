#pragma once
#include <stdint.h>
#define MAX_SIZE 32
// 队列结构体定义
typedef struct {
    float items[MAX_SIZE];
    int front;
    int rear;
    int size;
} Queue;

void initQueue(Queue *q);
void enqueue(Queue *q, float value);
float dequeue(Queue *q);
float peek(Queue *q);
float calQueueMean(Queue *q);