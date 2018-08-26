//
// Created by jbs on 18. 4. 9.
//

#ifndef IMAGE_TRACKING_CLION_SMOOTHING_FILTER_H
#define IMAGE_TRACKING_CLION_SMOOTHING_FILTER_H

#include <vector>
#include <iostream>
#include <deque>
#include <vector>
#include <cmath>
#include <algorithm>

struct Filter {
    Filter() {};
    Filter(int D,int N_buffer) {
        this->D=D; this->buffer_capacity=N_buffer;
        this->Buffer=std::vector<std::deque<double>>(D);
        buffer_size=0;
    };
    ~Filter() {};
    int D;
    int buffer_capacity;
    int buffer_size;
    std::vector<std::deque<double>> Buffer;
    std::vector<double> filter_output();
    void buffer_insert(std::vector<double> );
    void buffer_print();
};




#endif //IMAGE_TRACKING_CLION_SMOOTHING_FILTER_H
