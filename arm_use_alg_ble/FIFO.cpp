#include "FIFO.h"

FIFO::FIFO() : head(nullptr), tail(nullptr) {}

FIFO::~FIFO() {
    while (!isEmpty()) {
        unsigned long long tempEpoch;
        float tempFloat;
        pop(tempEpoch, tempFloat);
    }
}

void FIFO::push(unsigned long long epochValue, float floatValue) {
    Node* newNode = new Node();
    newNode->epochValue = epochValue;
    newNode->floatValue = floatValue;
    newNode->next = nullptr;

    if (tail) {
        tail->next = newNode;
    } else {
        head = newNode;
    }
    tail = newNode;
}

bool FIFO::pop(unsigned long long &epochValue, float &floatValue) {
    if (isEmpty()) {
        return false;
    }

    Node* tempNode = head;
    epochValue = head->epochValue;
    floatValue = head->floatValue;
    head = head->next;

    if (!head) {
        tail = nullptr;
    }

    delete tempNode;
    return true;
}

bool FIFO::isEmpty() {
    return head == nullptr;
}
