#include <zed_node/smoothing_filter.h>


void Filter::buffer_print() {

    for (int i=0;i<D;i++) {
        std::cout<<i<<"th dim: ";
        for (int j=0;j<buffer_size;j++)
            std::cout<<Buffer[i][j]<<" ";
        std::cout<<std::endl;
    }


}

// insert data to buffer
void Filter::buffer_insert(std::vector<double> input_data) {
    // full -> pop front
    if(buffer_capacity==buffer_size) {

        for (int i = 0; i < D; i++)
            Buffer[i].pop_front();
        buffer_size-=1;
    }

    for (int i=0;i<D;i++) {
        Buffer[i].push_back(input_data[i]);
    }
    buffer_size+=1;
}


std::vector<double> Filter::filter_output() {

    /**
     *  weights local
     */

    // generate weight_local : middle
    double local_weight_idx = floor(buffer_size / 2);
    std::deque<double> weight_local;
    for (int i = 0; i < buffer_size; i++)
        weight_local.push_back(pow(1 - pow((i - local_weight_idx) / buffer_size, 2), 3));

    // obtain mean value of each dimension
    std::vector<double> center;

    for (int i = 0; i < D; i++) {
        double mean_i = 0;
        mean_i = std::accumulate(Buffer[i].begin(), Buffer[i].end(), mean_i) / buffer_size;
        center.push_back(mean_i);
    }


    /**
     *  weights robust
     */

    // obtain residual
    std::deque<double> residual;
    for (int i = 0; i < buffer_size; i++) {
        double L2_dist = 0;
        for (int j = 0; j < D; j++)
            if (j > 0)
                L2_dist += pow(Buffer[j][i] - center[j], 2);
            else
                L2_dist += pow(Buffer[j][i] - center[j], 2);

        residual.push_back(sqrt(L2_dist));
    }

    std::deque<double> residual_sorted(residual);
    // meadian of residual
    std::sort(residual_sorted.begin(), residual_sorted.end());
    double median = residual_sorted[floor(buffer_size/2)];

    // calculate weight_robust
    std::deque<double> weight_robust;
    for (int i = 0; i < buffer_size; i++)
        weight_robust.push_back(residual[i] < 2 * median ? pow(1 - pow(residual[i]/(2*median), 2), 2) : 0);


    /**
     *  output
     */

    std::vector<double> output;

    for (int i = 0; i < D; i++) {
        double weighted_sum = 0;
        double sum_weight = 0;
        for (int j = 0; j < buffer_size; j++) {
            weighted_sum += weight_local[j] * weight_robust[j] * Buffer[i][j];
            sum_weight += weight_local[j] * weight_robust[j];
        }
        output.push_back(weighted_sum / sum_weight);
    }


    return output;

}


//int main() {
//
//    Filter filter(1,5);
//
//    std::vector<double> input(1);
//    input[0]=1.5;
//    filter.buffer_insert(input);
//
//    input[0]=20;
//    filter.buffer_insert(input);
//
//    input[0]=1.5;
//    filter.buffer_insert(input);
//
//    input[0]=1.5;
//    filter.buffer_insert(input);
//
//
//    filter.buffer_print();
//    std::cout<<filter.filter_output()[0]<<std::endl;
//
//    return 0;
//}
