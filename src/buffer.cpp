#include <fstream>
#include <deque>
#include "./simulator.hpp"
#include <bits/stdc++.h>

// Bitrate map
std::map<float, int> bitRates_map { { 10.0 , 0 }, { 40.0 , 1 }, { 100.0 , 2 }, { 400.0 , 3 }, {1000.0, 4} };

// Buffer element class
class buffer_element {

  public:
    friend class Buffer;
    buffer_element(int src, int dst, long long id, BitRate *bitRate, double time_arrival){
      this->src = src;
      this->dst = dst;
      this->bitRate = new BitRate(*bitRate);
      this->id = id;
      this->time_arrival = time_arrival;
      this->current_attempts = 1;
    }

    buffer_element(int src, int dst, long long id, BitRate *bitRate, double time_arrival, int attempts){
      this->src = src;
      this->dst = dst;
      this->bitRate = new BitRate(*bitRate);
      this->id = id;
      this->time_arrival = time_arrival;
      this->current_attempts = attempts;
    }

    ~buffer_element() {};

    int src;
    int dst;
    long long id;
    BitRate *bitRate;
    double time_arrival;
    int current_attempts;

    bool operator>(const buffer_element &e) const
    {
      return bitRate->getBitRate() > e.bitRate->getBitRate();
    }

    bool operator<(const buffer_element &e) const
    {
      return bitRate->getBitRate() < e.bitRate->getBitRate();
    }

};

// Buffer class
class Buffer {
  friend class bufer_element;
  public:

    Buffer(){
      this->elements = std::deque<buffer_element>();
      this->last_time = 0;
      this->poped = 0;
      this->pushed = 0;
      this->mean_size_time = 0;
      this->mean_service_time = 0;
    }

    void addElement(buffer_element new_element){
      this->elements.push_back(new_element);
    }

    void pop_front(){
      this->elements.pop_front();
    }

    int size(){
      return this->elements.size();
    }

    void clear(){
      this->elements.clear();
    }

    buffer_element *front(){
      return &(this->elements.front());
    }

    buffer_element *back(){
      return &(this->elements.back());
    }

    std::deque<buffer_element> elements;
    double last_time;
    double mean_size_time;
    double mean_service_time;
    double mean_attempts;

    // Number of connections popped from buffer (allocated succesfully)
    int poped;
    int pushed;

};

// Calculate BBP n/Buffer
double bandwidthBlockingProbability(double bitrate_count_total[5], 
                                   double bitrate_count_blocked[5],
                                   double mean_weight_bitrate[5])
    {
    double BBP = 0;
    double BP = 0;
    double total_weight = 0;

    for (int b = 0; b < 5; b++){
        if (bitrate_count_total[b] == 0) continue;
        BP = bitrate_count_blocked[b] / bitrate_count_total[b];
        BBP += mean_weight_bitrate[b] * BP;
        total_weight += mean_weight_bitrate[b];
    }

    return (BBP/total_weight);
}

// Calculate BBP w/buffer
double bandwidthBlockingProbabilityWBuffer(double bitrate_count_total[5], 
                                           std::deque<buffer_element> buffer,
                                           double mean_weight_bitrate[5])
    {
    double BBP = 0;
    double BP = 0;
    double total_weight = 0;

    double count_blocked[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

    for (int i = 0; i < buffer.size(); i++){
        count_blocked[bitRates_map[buffer[i].bitRate->getBitRate()]] += 1;
    }

    for (int b = 0; b < 5; b++){
        if (count_blocked[b] == 0) continue;
        BP = count_blocked[b] / bitrate_count_total[b];
        BBP += mean_weight_bitrate[b] * BP;
        total_weight += mean_weight_bitrate[b];
    }

    return (BBP/total_weight);
}

// Result to TXT
void resultsToFile(bool buffer_state, std::fstream &output, double BBP, double BP, int number_connections,
                   int lambda_index, double earlang, double bitrate_count_blocked[5], Buffer buffer, double last_time)
{

    // avgService only poped connections
    double avgService = buffer.mean_service_time/buffer.poped;

    // avgService all poped and blocked connections
    for (int buffElement = 0; buffElement < buffer.size(); buffElement++){
      buffer.poped++;
      buffer.mean_service_time += last_time - buffer.front()->time_arrival;
    }

    double avgServiceAll = buffer.mean_service_time/buffer.poped;
    double avgAttempts = buffer.mean_attempts/buffer.poped;
    double avgSize = buffer.mean_size_time/buffer.last_time;

    switch (buffer_state){
        case false:
            // output info to txt:
            output << "N/Buffer earlang index: " << lambda_index
                    << ", earlang: " << earlang
                    << ", general blocking: " << BP
                    << ", BBP: " << BBP                    
                    << '\n';
            break;
        case true:
            if (buffer.size() == 0) std::cout << "\nNo elements in buffer! :P\n";
            // output info to txt:
            output << "W/Buffer earlang index: " << lambda_index
                    << ", earlang: " << earlang
                    << ", BBP: " << BBP 
                    << ", general blocking: " << (buffer.size()/(number_connections)) 
                    << ", general blocking (original): " << BP
                    << ", buffer size: " << buffer.size() 
                    << ", reallocated: " << buffer.poped 
                    << ", Average try per allocated element: " << avgAttempts
                    << ", Average service time: " << avgService
                    << ", Average service time (ALL): " << avgServiceAll
                    << ", Average buffer size: " << avgSize
                    << '\n';
            break;
        }
}