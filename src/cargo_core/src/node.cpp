#include "cargo_core/node.hpp"

std::string cargo::Node::generate_global_name(const std::string &name) {
    if (name.find("/") == 0) {
        return name.substr(1);
    } else {
        return name;
    }
}

std::string cargo::Node::generate_local_name(const std::string &name) {
    if (name.find("/") == 0) {
        return this->get_name() + name;
    } else {
        return std::string(this->get_name()) + "/" + name;
    }
}
