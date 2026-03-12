#include <iostream>
#include "utils/config.hpp"

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: vision <config.yaml> [experiment_overlay.yaml]\n";
        return 1;
    }

    Config cfg(argv[1]);
    if (argc >= 3) {
        cfg.merge(argv[2]);
    }

    std::cout << "aausat6-vision starting\n";
    std::cout << "Config: " << cfg.path() << "\n";

    // TODO: initialise pipeline and stages

    std::cout << "Hello, AAUSAT6 Vision!\n";

    return 0;
}