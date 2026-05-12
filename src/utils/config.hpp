#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>

// Parses YAML and lets us use values
//   Config cfg("config/default.yaml");
//    int n = cfg.get<int>("orb.n_features", 1000);

class Config {
  public:
    Config() = default;

    explicit Config(const std::filesystem::path &path) {
        load(path);
    }

    void load(const std::filesystem::path &path) {
        if (!std::filesystem::exists(path)) {
            throw std::runtime_error("Config file not found: " + path.string());
        }
        root_ = YAML::LoadFile(path.string());
        path_ = path;
    }

    //merge to use overlays
    void merge(const std::filesystem::path &overlay_path) {
        if (!std::filesystem::exists(overlay_path)) {
            throw std::runtime_error("Overlay config not found: " + overlay_path.string());
        }
        YAML::Node overlay = YAML::LoadFile(overlay_path.string());
        merge_nodes(root_, overlay);
    }

    //Get value via dots
    template <typename T>
    T get(const std::string &key_path, const T &default_val) const {
        YAML::Node node = resolve(key_path);
        if (!node || !node.IsDefined())
            return default_val;
        try {
            return node.as<T>();
        } catch (const YAML::Exception &) {
            return default_val;
        }
    }

    // required values
    template <typename T>
    T require(const std::string &key_path) const {
        YAML::Node node = resolve(key_path);
        if (!node || !node.IsDefined()) {
            throw std::runtime_error("Config: required key '" + key_path + "' is missing");
        }
        return node.as<T>();
    }

    bool has(const std::string &key_path) const {
        YAML::Node node = resolve(key_path);
        return node && node.IsDefined();
    }

    const std::filesystem::path &path() const { return path_; }

  private:
    YAML::Node resolve(const std::string &key_path) const {
        YAML::Node  node = YAML::Clone(root_);
        std::string segment;
        for (char c : key_path) {
            if (c == '.') {
                if (!node[segment])
                    return YAML::Node(YAML::NodeType::Undefined);
                node = node[segment];
                segment.clear();
            } else {
                segment += c;
            }
        }
        if (!segment.empty()) {
            if (!node[segment])
                return YAML::Node(YAML::NodeType::Undefined);
            node = node[segment];
        }
        return node;
    }

    static void merge_nodes(YAML::Node base, const YAML::Node &overlay) {
        if (!overlay.IsMap())
            return;
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