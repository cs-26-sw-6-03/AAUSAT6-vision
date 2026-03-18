#include "picture_db.hpp"

#include <algorithm>
#include <stdexcept>

PictureDB::PictureDB(std::filesystem::path path_to_pictures)
    : pictures_path_(std::move(path_to_pictures))
{}

cv::Mat PictureDB::get_raw_frame(std::filesystem::path relative_path) {
    auto full_path = pictures_path_ / relative_path;
    cv::Mat img = cv::imread(full_path.string());
    if (img.empty()) {
        throw std::runtime_error("PictureDB: failed to load image: " + full_path.string());
    }
    return img;
}

void PictureDB::refresh(cv::Ptr<cv::ORB> orb) {
    // Build a snapshot of {path -> mtime} for all current files on disk.
    // This directory scan is cheap (no image loading).
    std::map<std::filesystem::path, std::filesystem::file_time_type> current_mtimes;
    for (const auto& entry : std::filesystem::recursive_directory_iterator(pictures_path_)) {
        if (!entry.is_regular_file()) continue;
        auto relative = std::filesystem::relative(entry.path(), pictures_path_);
        current_mtimes[relative] = entry.last_write_time();
    }

    // Fast path: nothing added, removed, or modified since last scan.
    if (current_mtimes == file_mtime_map_) return;

    // Find removed or modified files (treat modification as remove + re-add).
    std::vector<std::filesystem::path> to_remove;
    for (const auto& [path, idx] : file_index_map_) {
        auto it = current_mtimes.find(path);
        if (it == current_mtimes.end() || it->second != file_mtime_map_[path])
            to_remove.push_back(path);
    }

    if (!to_remove.empty()) {
        for (const auto& path : to_remove)
            file_index_map_.erase(path);

        // Sort remaining entries by old index to preserve order when compacting.
        std::vector<std::pair<size_t, std::filesystem::path>> indexed;
        indexed.reserve(file_index_map_.size());
        for (const auto& [path, idx] : file_index_map_)
            indexed.emplace_back(idx, path);
        std::sort(indexed.begin(), indexed.end());

        std::vector<std::vector<cv::KeyPoint>> new_kps;
        std::vector<cv::Mat> new_desc;
        file_index_map_.clear();
        for (auto& [old_idx, path] : indexed) {
            file_index_map_[path] = new_kps.size();
            new_kps.push_back(std::move(keypoints_[old_idx]));
            new_desc.push_back(std::move(descriptors_[old_idx]));
        }
        keypoints_   = std::move(new_kps);
        descriptors_ = std::move(new_desc);
    }

    // Add new (or re-add modified) files.
    for (const auto& [relative, mtime] : current_mtimes) {
        if (file_index_map_.count(relative)) continue;

        cv::Mat img = cv::imread((pictures_path_ / relative).string());
        if (img.empty()) continue;

        std::vector<cv::KeyPoint> kps;
        cv::Mat desc;
        orb->detectAndCompute(img, cv::noArray(), kps, desc);

        file_index_map_[relative] = keypoints_.size();
        keypoints_.push_back(std::move(kps));
        descriptors_.push_back(std::move(desc));
    }

    file_mtime_map_ = std::move(current_mtimes);
}

int PictureDB::index_of(const std::filesystem::path& relative_path) const {
    auto it = file_index_map_.find(relative_path);
    if (it == file_index_map_.end()) return -1;
    return static_cast<int>(it->second);
}
