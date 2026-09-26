#include <iostream>
#include <atomic>


int main() {
    std::atomic<int> counter(0);

    counter++;

    std::cout << "Finish counter: " << counter <<  std::endl;

    return 1;
    
} 