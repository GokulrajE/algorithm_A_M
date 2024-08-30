#ifndef FIFO_H
#define FIFO_H

#include <Arduino.h>

class FIFO {
public:
    FIFO();
    ~FIFO();

    void push(unsigned long long epochValue, float floatValue);
    bool pop(unsigned long long &epochValue, float &floatValue);
    bool isEmpty();

private:
    struct Node {
        unsigned long long epochValue;
        float floatValue;
        Node* next;
    };

    Node* head;
    Node* tail;
};

#endif
