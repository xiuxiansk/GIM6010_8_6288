#include "my_queue.h"

// 初始化队列
void initQueue(Queue *q)
{
    q->front = 0;
    q->rear  = -1;
    q->size  = 0;
}

// 入队操作
void enqueue(Queue *q, float value)
{
    if (q->size == MAX_SIZE) {
        // printf("队列已满，无法插入元素\n");
    } else {
        q->rear           = (q->rear + 1) % MAX_SIZE;
        q->items[q->rear] = value;
        q->size++;
    }
}

// 出队操作
float dequeue(Queue *q)
{
    if (q->size == 0) {
        // printf("队列已空，无法删除元素\n");
        return -1;
    } else {
        int dequeued = q->items[q->front];
        q->front     = (q->front + 1) % MAX_SIZE;
        q->size--;
        return dequeued;
    }
}

// 查看队头元素
float peek(Queue *q)
{
    if (q->size == 0) {
        // printf("队列已空\n");
        return -1;
    } else {
        return q->items[q->front];
    }
}
// 计算队列中元素的均值
float calQueueMean(Queue *q)
{
    if (q->size == 0) {
        // 如果队列为空，返回 0 或者其他适当的值
        return 0.0f;
    }
    float sum = 0;

    // 从 front 到 rear 遍历队列元素
    for (int i = 0; i < q->size; i++) {
        sum += q->items[(q->front + i) % MAX_SIZE];
    }

    // 返回均值
    return (float)sum / q->size;
}
