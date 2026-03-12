/* 
 * config.hpp
 *
 * Load the config form a YAML file
 * TODO: Needs review
 */
#pragma once

#include <yaml-cpp/yaml.h>
#include <string>
#include <stdexcept>
#include <filesystem>

// Wraps a YAML::Node and provides typed accessors with defaults.
// Loaded once at startup and passed as const ref or shared_ptr to stages.
//
// Usage:
//   Config cfg("config/default.yaml");
//   int n = cfg.get<int>("orb.n_features", 1000);
class Config {
public:
    Config() = default;

    explicit Config(const std::filesystem::path& path) {
        load(path);
    }

    void load(const std::filesystem::path& path) {
        if (!std::filesystem::exists(path)) {
            throw std::runtime_error("Config file not found: " + path.string());
        }
        root_ = YAML::LoadFile(path.string());
        path_  = path;
    }

    // Merge another config on top — values in overlay take precedence.
    // Useful for experiment configs that only override specific keys.
    void merge(const std::filesystem::path& overlay_path) {
        if (!std::filesystem::exists(overlay_path)) {
            throw std::runtime_error("Overlay config not found: " + overlay_path.string());
        }
        YAML::Node overlay = YAML::LoadFile(overlay_path.string());
        merge_nodes(root_, overlay);
    }

    // Get a value by dot-separated key path, e.g. "orb.n_features"
    // Returns default_val if the key does not exist.
    template <typename T>
    T get(const std::string& key_path, const T& default_val) const {
        YAML::Node node = resolve(key_path);
        if (!node || !node.IsDefined()) return default_val;
        try {
            return node.as<T>();
        } catch (const YAML::Exception&) {
            return default_val;
        }
    }

    // Get a required value — throws if missing
    template <typename T>
    T require(const std::string& key_path) const {
        YAML::Node node = resolve(key_path);
        if (!node || !node.IsDefined()) {
            throw std::runtime_error("Config: required key '" + key_path + "' is missing");
        }
        return node.as<T>();
    }

    bool has(const std::string& key_path) const {
        YAML::Node node = resolve(key_path);
        return node && node.IsDefined();
    }

    const std::filesystem::path& path() const { return path_; }

private:
    YAML::Node resolve(const std::string& key_path) const {
        YAML::Node node = YAML::Clone(root_);
        std::string segment;
        for (char c : key_path) {
            if (c == '.') {
                if (!node[segment]) return YAML::Node(YAML::NodeType::Undefined);
                node = node[segment];
                segment.clear();
            } else {
                segment += c;
            }
        }
        if (!segment.empty()) {
            if (!node[segment]) return YAML::Node(YAML::NodeType::Undefined);
            node = node[segment];
        }
        return node;
    }

    static void merge_nodes(YAML::Node base, const YAML::Node& overlay) {
        if (!overlay.IsMap()) return;
        for (auto it = overlay.begin(); it != overlay.end(); ++it) {
            const std::string key = it->first.as<std::string>();
            if (base[key] && base[key].IsMap() && it->second.IsMap()) {
                merge_nodes(base[key], it->second);
            } else {
                base[key] = it->second;
            }
        }
    }

    YAML::Node            root_;
    std::filesystem::path path_;
};