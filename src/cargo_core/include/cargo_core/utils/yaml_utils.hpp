#pragma once

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <bitset>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace cargo {
namespace yaml {

std::filesystem::path
get_project_export_path_from_xml_path(const std::filesystem::path &xml_path);
std::vector<std::filesystem::path>
find_yaml_files_inside(const std::filesystem::path &dir);

YAML::Node find_tag_across_multiple_yaml_files(
    const std::vector<std::filesystem::path> &yaml_files,
    const std::string &tag);

uint8_t parse_uint_from_string(const std::string &str);
std::vector<uint8_t>
parse_uint_from_string(const std::vector<std::string> &str_v);

// Recursive function to find the tag inside the yaml file
YAML::Node find_tag_in_yaml_node(const YAML::Node &node,
                                 const std::string &tag);

template <typename T = std::string>
std::vector<T> find_tag_in_yaml_file(const std::filesystem::path &yaml_file,
                                     const std::string &tag) {
    std::vector<T> result;
    std::ifstream config_file(yaml_file);
    // check if file exists
    if (!config_file.good()) {
        throw std::runtime_error("Could not open yaml file: " +
                                 yaml_file.string());
    }
    YAML::Node node = YAML::LoadFile(yaml_file.string());
    if (!node.IsNull()) {
        YAML::Node tag_node = find_tag_in_yaml_node(node, tag);
        if (!tag_node.IsNull()) {
            // for (const auto & tag_value : tag_node) {
            //   result.push_back(tag_value.as<T>()); }
            // done it with std::transform ->
            std::transform(tag_node.begin(), tag_node.end(),
                           std::back_inserter(result),
                           [](const YAML::Node &node) { return node.as<T>(); });
        }
    } else {
        throw std::runtime_error("Could not parse yaml file: " +
                                 yaml_file.string());
    }

    return result;
}

template <typename T = std::string>
std::vector<T> find_tag_from_project_exports_path(
    const std::filesystem::path &project_exports_path, const std::string &tag) {
    std::vector<T> values;
    std::vector<std::filesystem::path> yaml_files =
        find_yaml_files_inside(project_exports_path);
    if (yaml_files.size() == 0) {
        throw std::runtime_error("Could not find any yaml files in: " +
                                 project_exports_path.string());
    }

    for (const auto &yaml_file : yaml_files) {
        auto tags = find_tag_in_yaml_file<T>(yaml_file, tag);
        if (tags.size() > 0) {
            values = tags;
        }
    }
    return values;
}

} // namespace yaml
} // namespace cargo
