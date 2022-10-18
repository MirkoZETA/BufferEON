#include "./src/buffer.cpp"
/*********************************************************************************
 * This is an allocation algorithm that follows the First Fit allocation
 * policy and include the implementation of a infinite buffer that stores blocked
 * connections and try to allocate them every time a departure occurs, also using
 * First Fit. This policy always chooses the available slots with the lowest posible
 * index to serve the connection request. If the required slot or group of slots
 * is available, taking into account the spectrum contiguity and continuity
 * constraints, it creates the connections and returns 'ALLOCATED' to indicate
 * success; otherwise, it returns 'NOT_ALLOCATED' to indicate that the process
 * failed and add it to the buffer (or keep in it if allocating from buffer)
 **********************************************************************************/


// ############################## Global Variables #################################

// Queue for buffer
Buffer buffer;

// BBP global variables
double bitrate_count_total[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
double bitrate_count_blocked[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

// Buffer state
bool buffer_state = true;
bool allocating_from_buffer = false;

// Weight RMSA:
//double mean_weight_bitrate[5] = {1.0, 1.25, 3.0, 9.5, 23};

// Weight RSA:
double mean_weight_bitrate[5] = {1.0, 2.0, 5.0, 18.0, 23.0};

// Controller and Simulator for acces from UNALLOC
Controller *buffer_controller;
Simulator sim;

// Variables for output of times every connection is allocated from buffer
std::fstream realloc_time;
const char* fileName[21] = {
                    "./realloc_times/1_","./realloc_times/2_","./realloc_times/3_",
                    "./realloc_times/4_","./realloc_times/5_","./realloc_times/6_",
                    "./realloc_times/7_","./realloc_times/8_","./realloc_times/9_",
                    "./realloc_times/10_","./realloc_times/11_","./realloc_times/12_",
                    "./realloc_times/13_","./realloc_times/14_","./realloc_times/15_",
                    "./realloc_times/16_","./realloc_times/17_","./realloc_times/18_",
                    "./realloc_times/19_","./realloc_times/20_","./realloc_times/21_"
                    };
const char* concatenate[6] = {
                    "NSFnet_RSA.txt", "COST239_RSA.txt", "USNet_RSA.txt", "UKNet_RSA.txt", "EuroCore_RSA.txt", "ARPANet_RSA.txt"
                    };
char file[30];

// Save last time in simulation
double last_time = 0;

// #################################################################################

// Allocation function
BEGIN_ALLOC_FUNCTION(FirstFit) {

  last_time = con.getTimeConnection();

  // Calculate avg buffer size
  if (buffer_state){
    buffer.mean_size_time += buffer.size()*(con.getTimeConnection() -  buffer.last_time);
    buffer.last_time = con.getTimeConnection();
  }

  int currentNumberSlots;
  int currentSlotIndex;
  int bitRateInt = bitRates_map[REQ_BITRATE];
  int numberOfSlots;

  if (!allocating_from_buffer) bitrate_count_total[bitRateInt] += 1;
  std::vector<bool> totalSlots;
  for (int r = 0; r < NUMBER_OF_ROUTES;
        r++){ // <- For route r between current SRC and DST
    totalSlots = std::vector<bool>(LINK_IN_ROUTE(r, 0)->getSlots(), false); // <- Assuming all links have the same capacity
    for (int m = 0; m < NUMBER_OF_MODULATIONS;
        m++){ // <- For modulation m
        numberOfSlots = REQ_SLOTS(m); //<- Number of slots that this modulation format requires
        double route_length = 0; // For checking route length
        for (int l = 0; l < NUMBER_OF_LINKS(r);
            l++){ // <- this loops through the links that the current route contains

          route_length += LINK_IN_ROUTE(r,l)->getLength();

          for (int s = 0; s < LINK_IN_ROUTE(r, l)->getSlots();
              s++){   // <- this loops through the slots on the current link to fill
                      //    the total slots vector with the slot status information
            totalSlots[s] = totalSlots[s] | LINK_IN_ROUTE(r, l)->getSlot(s);
            }
          }
          // We verify that the current modulation format has sufficient reach
          if (route_length > REQ_REACH(m))
          {
            // std::cout << "Not reaching!\n";
            continue;
          }
          // Number of consecutive free slots:
          currentNumberSlots = 0;
          currentSlotIndex = 0;
          for (int s = 0; s < totalSlots.size();
              s++) {  // this loops through the totalSlots vector to find if the
                      // required amount of continous slots is available on all
                      // the links of the current route. If the value of a position
                      // on the vector is false, the corresponding slot is available
                      // on every link on the route; otherwise, it's unavailable
              if (totalSlots[s] == false) {
                currentNumberSlots++;
              } else {
                currentNumberSlots = 0;
                currentSlotIndex = s + 1;
              }
              if (currentNumberSlots == numberOfSlots) {
                for (int l = 0; l < NUMBER_OF_LINKS(r);
                    l++) {  // if the necessary amount of consecutive slots was
                            // available, this loops through the links on that route to
                            // create the connections
                  ALLOC_SLOTS(LINK_IN_ROUTE_ID(r, l), currentSlotIndex, numberOfSlots)
                }
                return ALLOCATED;
              }
          }
      }
    }
    bitrate_count_blocked[bitRateInt] += 1;

    if (buffer_state){
      // If the present connection ISN'T coming from buffer, we push to queue
      if (!allocating_from_buffer){
        buffer.addElement(buffer_element(SRC, DST, con.getId(), con.getBitrate(), con.getTimeConnection()));
      }
      // If the present connection IS coming from buffer, add another attempt
      else {
        buffer.front()->current_attempts++;
      }
    }

    return NOT_ALLOCATED;
}
END_ALLOC_FUNCTION

// Unalloc callback function
BEGIN_UNALLOC_CALLBACK_FUNCTION {
  if (buffer.size() > 0){

    // For simplicity
    buffer_element *front_queue = buffer.front();

    // Let the alloc function know we are allocating from buffer
    allocating_from_buffer = true;

    // try to alloc
    if (buffer_controller->assignConnection(front_queue->src, front_queue->dst, *(front_queue->bitRate), front_queue->id, t) == ALLOCATED){

      // Add departure to event routine
      sim.addDepartureEvent(front_queue->id);

      // Total time the connection was in queue
      buffer.mean_service_time += t - front_queue->time_arrival;

      // We keep track of how many times attempted to be allocated from buffer
      buffer.mean_attempts += buffer.front()->current_attempts;

      // We output the currrent time being allocated
      realloc_time << t << "\n";
      //std::cout << t << "\n";
      
      // Element allocated so we poped it and delete() members
      delete(buffer.front()->bitRate);
      buffer.pop_front();

      // Keep count of how many connections where allocated from the buffer
      buffer.poped++;
      // std::cout << "Allocated!\n";   
    }

    // Not allocating from buffer anymore
    allocating_from_buffer = false;
  }
}
END_UNALLOC_CALLBACK_FUNCTION

int main(int argc, char* argv[]) {

  // Sim parameters
  double  lambdas[21] = {180, 216, 252, 288, 324, 360, 396, 432, 468, 504, 540, 576, 612, 648, 684, 720, 756, 792, 828, 864, 900};
  double mu = 1;
  int number_connections = 1e7;

  // ############################## NSFNET #################################
  for (int lambda = 0; lambda < sizeof(lambdas)/sizeof(double); lambda++) {

    // Buffer state to console (ON/OFF)
    if (buffer_state) std::cout << "Buffer:\t\t    ON\n";
    else std::cout << "Buffer:\t\t    OFF\n";

    // Simulator object
    sim = Simulator(std::string("./networks/NSFNet.json"),                      // Network nodes and links
                    std::string("./networks/NSFNet_routes.json"),               // Network Routes
                    std::string("./networks/bitrates_RSA.json"));       // BitRates and bands (eg. BPSK/C)

    // Assign alloc function   
    USE_ALLOC_FUNCTION(FirstFit, sim);

    // Assign unalloc function ONLY if buffer is activated
    if (buffer_state){
      USE_UNALLOC_FUNCTION(sim);
      // Output of realloc times:
      strncpy(file, "", sizeof(file));
      strcat(file, fileName[lambda]);
      strcat(file, concatenate[0]);
      std::cout << file << "\n";
      realloc_time.open(file, std::ios::out | std::ios::app);
    }

    // Assign parameters
    sim.setGoalConnections(number_connections);
    sim.setLambda(lambdas[lambda]);
    sim.setMu(mu);
    sim.init();

    // Set controller accessible for unalloc function (required for buffer)
    buffer_controller = sim.getController();

    // Begin simulation
    sim.run();

    // BBP calculation
    double BBP_results;
      // different BBP formula depending if buffer is activated
    if (buffer_state) BBP_results = bandwidthBlockingProbabilityWBuffer(bitrate_count_total, buffer.elements, mean_weight_bitrate);
    else BBP_results = bandwidthBlockingProbability(bitrate_count_total, bitrate_count_blocked, mean_weight_bitrate);

    // Output results to TXT
    std::fstream output;
    output.open("./out/RSA-NSFNet-NBuffer-1e7.txt", std::ios::out | std::ios::app);

    resultsToFile(buffer_state, output, BBP_results, sim.getBlockingProbability(), number_connections,
                  lambda, lambdas[lambda], bitrate_count_blocked, buffer, last_time);

    if (buffer_state) {
      realloc_time.close();
      strncpy(file, "", sizeof(file));
    }

    // Reset global variables
      // Clear buffer and related variables
    for (int be = 0; be < buffer.size(); be++) delete(buffer.elements[be].bitRate);
    buffer.clear();
    buffer.poped = 0;
    buffer.last_time = 0;
    buffer.mean_service_time = 0;
    buffer.mean_size_time = 0;
    buffer.mean_attempts = 0;
      // Reset global variables for BBP calculation
    for (int b = 0; b < 5; b++){
      bitrate_count_total[b] = 0.0;
      bitrate_count_blocked[b] = 0.0;
    }
    last_time = 0;

  }

  // ############################## COST239 #################################
  for (int lambda = 0; lambda < sizeof(lambdas)/sizeof(double); lambda++) {

    // Buffer state to console (ON/OFF)
    if (buffer_state) std::cout << "Buffer:\t\t    ON\n";
    else std::cout << "Buffer:\t\t    OFF\n";

    // Simulator object
    sim = Simulator(std::string("./networks/COST239.json"),                      // Network nodes and links
                    std::string("./networks/COST239_routes.json"),               // Network Routes
                    std::string("./networks/bitrates_RSA.json"));       // BitRates and bands (eg. BPSK/C)

    // Assign alloc function   
    USE_ALLOC_FUNCTION(FirstFit, sim);

    // Assign unalloc function ONLY if buffer is activated
    if (buffer_state){
      USE_UNALLOC_FUNCTION(sim);
      // Output of realloc times:
      strncpy(file, "", sizeof(file));
      strcat(file, fileName[lambda]);
      strcat(file, concatenate[1]);
      realloc_time.open(file, std::ios::out | std::ios::app);
    }

    // Assign parameters
    sim.setGoalConnections(number_connections);
    sim.setLambda(lambdas[lambda]);
    sim.setMu(mu);
    sim.init();

    // Set controller accessible for unalloc function (required for buffer)
    buffer_controller = sim.getController();

    // Begin simulation
    sim.run();

    // BBP calculation
    double BBP_results;
      // different BBP formula depending if buffer is activated
    if (buffer_state) BBP_results = bandwidthBlockingProbabilityWBuffer(bitrate_count_total, buffer.elements, mean_weight_bitrate);
    else BBP_results = bandwidthBlockingProbability(bitrate_count_total, bitrate_count_blocked, mean_weight_bitrate);

    // Output results to TXT
    std::fstream output;
    output.open("./out/RSA-COST239-NBuffer-1e7.txt", std::ios::out | std::ios::app);

    resultsToFile(buffer_state, output, BBP_results, sim.getBlockingProbability(), number_connections,
                  lambda, lambdas[lambda], bitrate_count_blocked, buffer, last_time);

    if (buffer_state) {
      realloc_time.close();
      strncpy(file, "", sizeof(file));
    }

    // Reset global variables
      // Clear buffer and related variables
    for (int be = 0; be < buffer.size(); be++) delete(buffer.elements[be].bitRate);
    buffer.clear();
    buffer.poped = 0;
    buffer.last_time = 0;
    buffer.mean_service_time = 0;
    buffer.mean_size_time = 0;
    buffer.mean_attempts = 0;
      // Reset global variables for BBP calculation
    for (int b = 0; b < 5; b++){
      bitrate_count_total[b] = 0.0;
      bitrate_count_blocked[b] = 0.0;
    }
    last_time = 0;
  }

  // ############################## USNet #################################
  for (int lambda = 0; lambda < sizeof(lambdas)/sizeof(double); lambda++) {

    // Buffer state to console (ON/OFF)
    if (buffer_state) std::cout << "Buffer:\t\t    ON\n";
    else std::cout << "Buffer:\t\t    OFF\n";

    // Simulator object
    sim = Simulator(std::string("./networks/USNet.json"),                      // Network nodes and links
                    std::string("./networks/USNet_routes.json"),               // Network Routes
                    std::string("./networks/bitrates_RSA.json"));       // BitRates and bands (eg. BPSK/C)

    // Assign alloc function   
    USE_ALLOC_FUNCTION(FirstFit, sim);

    // Assign unalloc function ONLY if buffer is activated
    if (buffer_state){
      USE_UNALLOC_FUNCTION(sim);
      // Output of realloc times:
      strncpy(file, "", sizeof(file));
      strcat(file, fileName[lambda]);
      strcat(file, concatenate[2]);
      realloc_time.open(file, std::ios::out | std::ios::app);
    }

    // Assign parameters
    sim.setGoalConnections(number_connections);
    sim.setLambda(lambdas[lambda]);
    sim.setMu(mu);
    sim.init();

    // Set controller accessible for unalloc function (required for buffer)
    buffer_controller = sim.getController();

    // Begin simulation
    sim.run();

    // BBP calculation
    double BBP_results;
      // different BBP formula depending if buffer is activated
    if (buffer_state) BBP_results = bandwidthBlockingProbabilityWBuffer(bitrate_count_total, buffer.elements, mean_weight_bitrate);
    else BBP_results = bandwidthBlockingProbability(bitrate_count_total, bitrate_count_blocked, mean_weight_bitrate);

    // Output results to TXT
    std::fstream output;
    output.open("./out/RSA-USNet-NBuffer-1e7.txt", std::ios::out | std::ios::app);

    resultsToFile(buffer_state, output, BBP_results, sim.getBlockingProbability(), number_connections,
                  lambda, lambdas[lambda], bitrate_count_blocked, buffer, last_time);

    if (buffer_state) {
      realloc_time.close();
      strncpy(file, "", sizeof(file));
    }

    // Reset global variables
      // Clear buffer and related variables
    for (int be = 0; be < buffer.size(); be++) delete(buffer.elements[be].bitRate);
    buffer.clear();
    buffer.poped = 0;
    buffer.last_time = 0;
    buffer.mean_service_time = 0;
    buffer.mean_size_time = 0;
    buffer.mean_attempts = 0;
      // Reset global variables for BBP calculation
    for (int b = 0; b < 5; b++){
      bitrate_count_total[b] = 0.0;
      bitrate_count_blocked[b] = 0.0;
    }
    last_time = 0;

  }

  // ############################## UKNet #################################
  for (int lambda = 0; lambda < sizeof(lambdas)/sizeof(double); lambda++) {

    // Buffer state to console (ON/OFF)
    if (buffer_state) std::cout << "Buffer:\t\t    ON\n";
    else std::cout << "Buffer:\t\t    OFF\n";

    // Simulator object
    sim = Simulator(std::string("./networks/UKNet.json"),                      // Network nodes and links
                    std::string("./networks/UKNet_routes.json"),               // Network Routes
                    std::string("./networks/bitrates_RSA.json"));       // BitRates and bands (eg. BPSK/C)

    // Assign alloc function   
    USE_ALLOC_FUNCTION(FirstFit, sim);

    // Assign unalloc function ONLY if buffer is activated
    if (buffer_state){
      // Output of realloc times:
      strncpy(file, "", sizeof(file));
      strcat(file, fileName[lambda]);
      strcat(file, concatenate[3]);
      realloc_time.open(file, std::ios::out | std::ios::app);
    }

    // Assign parameters
    sim.setGoalConnections(number_connections);
    sim.setLambda(lambdas[lambda]);
    sim.setMu(mu);
    sim.init();

    // Set controller accessible for unalloc function (required for buffer)
    buffer_controller = sim.getController();

    // Begin simulation
    sim.run();

    // BBP calculation
    double BBP_results;
      // different BBP formula depending if buffer is activated
    if (buffer_state) BBP_results = bandwidthBlockingProbabilityWBuffer(bitrate_count_total, buffer.elements, mean_weight_bitrate);
    else BBP_results = bandwidthBlockingProbability(bitrate_count_total, bitrate_count_blocked, mean_weight_bitrate);

    // Output results to TXT
    std::fstream output;
    output.open("./out/RSA-UKNet-NBuffer-1e7.txt", std::ios::out | std::ios::app);

    resultsToFile(buffer_state, output, BBP_results, sim.getBlockingProbability(), number_connections,
                  lambda, lambdas[lambda], bitrate_count_blocked, buffer, last_time);

    if (buffer_state) {
      realloc_time.close();
      strncpy(file, "", sizeof(file));
    }

    // Reset global variables
      // Clear buffer and related variables
    for (int be = 0; be < buffer.size(); be++) delete(buffer.elements[be].bitRate);
    buffer.clear();
    buffer.poped = 0;
    buffer.last_time = 0;
    buffer.mean_service_time = 0;
    buffer.mean_size_time = 0;
    buffer.mean_attempts = 0;
      // Reset global variables for BBP calculation
    for (int b = 0; b < 5; b++){
      bitrate_count_total[b] = 0.0;
      bitrate_count_blocked[b] = 0.0;
    }
    last_time = 0;
  }

  // ############################## EUROCORE #################################
  for (int lambda = 0; lambda < sizeof(lambdas)/sizeof(double); lambda++) {

    // Buffer state to console (ON/OFF)
    if (buffer_state) std::cout << "Buffer:\t\t    ON\n";
    else std::cout << "Buffer:\t\t    OFF\n";

    // Simulator object
    sim = Simulator(std::string("./networks/EuroCore.json"),                      // Network nodes and links
                    std::string("./networks/EuroCore_routes.json"),               // Network Routes
                    std::string("./networks/bitrates_RSA.json"));       // BitRates and bands (eg. BPSK/C)

    // Assign alloc function   
    USE_ALLOC_FUNCTION(FirstFit, sim);

    // Assign unalloc function ONLY if buffer is activated
    if (buffer_state){
      USE_UNALLOC_FUNCTION(sim);
      // Output of realloc times:
      strncpy(file, "", sizeof(file));
      strcat(file, fileName[lambda]);
      strcat(file, concatenate[4]);
      realloc_time.open(file, std::ios::out | std::ios::app);
    }

    // Assign parameters
    sim.setGoalConnections(number_connections);
    sim.setLambda(lambdas[lambda]);
    sim.setMu(mu);
    sim.init();

    // Set controller accessible for unalloc function (required for buffer)
    buffer_controller = sim.getController();

    // Begin simulation
    sim.run();

    // BBP calculation
    double BBP_results;
      // different BBP formula depending if buffer is activated
    if (buffer_state) BBP_results = bandwidthBlockingProbabilityWBuffer(bitrate_count_total, buffer.elements, mean_weight_bitrate);
    else BBP_results = bandwidthBlockingProbability(bitrate_count_total, bitrate_count_blocked, mean_weight_bitrate);

    // Output results to TXT
    std::fstream output;
    output.open("./out/RSA-EuroCore-NBuffer-1e7.txt", std::ios::out | std::ios::app);

    resultsToFile(buffer_state, output, BBP_results, sim.getBlockingProbability(), number_connections,
                  lambda, lambdas[lambda], bitrate_count_blocked, buffer, last_time);

    if (buffer_state) {
      realloc_time.close();
      strncpy(file, "", sizeof(file));
    }

    // Reset global variables
      // Clear buffer and related variables
    for (int be = 0; be < buffer.size(); be++) delete(buffer.elements[be].bitRate);
    buffer.clear();
    buffer.poped = 0;
    buffer.last_time = 0;
    buffer.mean_service_time = 0;
    buffer.mean_size_time = 0;
    buffer.mean_attempts = 0;
      // Reset global variables for BBP calculation
    for (int b = 0; b < 5; b++){
      bitrate_count_total[b] = 0.0;
      bitrate_count_blocked[b] = 0.0;
    }
    last_time = 0;
  }

  // ############################## ARPANet #################################
  for (int lambda = 0; lambda < sizeof(lambdas)/sizeof(double); lambda++) {

    // Buffer state to console (ON/OFF)
    if (buffer_state) std::cout << "Buffer:\t\t    ON\n";
    else std::cout << "Buffer:\t\t    OFF\n";

    // Simulator object
    sim = Simulator(std::string("./networks/ARPANet.json"),                      // Network nodes and links
                    std::string("./networks/ARPANet_routes.json"),               // Network Routes
                    std::string("./networks/bitrates_RSA.json"));       // BitRates and bands (eg. BPSK/C)

    // Assign alloc function   
    USE_ALLOC_FUNCTION(FirstFit, sim);

    // Assign unalloc function ONLY if buffer is activated
    if (buffer_state){
      USE_UNALLOC_FUNCTION(sim);
      // Output of realloc times:
      strncpy(file, "", sizeof(file));
      strcat(file, fileName[lambda]);
      strcat(file, concatenate[5]);
      realloc_time.open(file, std::ios::out | std::ios::app);
    }

    // Assign parameters
    sim.setGoalConnections(number_connections);
    sim.setLambda(lambdas[lambda]);
    sim.setMu(mu);
    sim.init();

    // Set controller accessible for unalloc function (required for buffer)
    buffer_controller = sim.getController();

    // Begin simulation
    sim.run();

    // BBP calculation
    double BBP_results;
      // different BBP formula depending if buffer is activated
    if (buffer_state) BBP_results = bandwidthBlockingProbabilityWBuffer(bitrate_count_total, buffer.elements, mean_weight_bitrate);
    else BBP_results = bandwidthBlockingProbability(bitrate_count_total, bitrate_count_blocked, mean_weight_bitrate);

    // Output results to TXT
    std::fstream output;
    output.open("./out/RSA-ARPANet-NBuffer-1e7.txt", std::ios::out | std::ios::app);

    resultsToFile(buffer_state, output, BBP_results, sim.getBlockingProbability(), number_connections,
                  lambda, lambdas[lambda], bitrate_count_blocked, buffer, last_time);

    if (buffer_state) {
      realloc_time.close();
      strncpy(file, "", sizeof(file));
    }

    // Reset global variables
      // Clear buffer and related variables
    for (int be = 0; be < buffer.size(); be++) delete(buffer.elements[be].bitRate);
    buffer.clear();
    buffer.poped = 0;
    buffer.last_time = 0;
    buffer.mean_service_time = 0;
    buffer.mean_size_time = 0;
    buffer.mean_attempts = 0;
      // Reset global variables for BBP calculation
    for (int b = 0; b < 5; b++){
      bitrate_count_total[b] = 0.0;
      bitrate_count_blocked[b] = 0.0;
    }
    last_time = 0;
  }
  return 0;
}