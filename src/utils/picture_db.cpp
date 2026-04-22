#include "picture_db.hpp"

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
    for (const auto& entry : std::filesystem::recursive_directory_iterator(pictures_path_)) {
        if (!entry.is_regular_file()) continue;

        auto relative = std::filesystem::relative(entry.path(), pictures_path_);
        if (file_index_map_.count(relative)) continue;  // already loaded

        cv::Mat img = cv::imread(entry.path().string());
        if (img.empty()) continue;  // skip unreadable files

        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::KeyPoint> kps;
        cv::Mat desc;
        orb->detectAndCompute(gray, cv::noArray(), kps, desc);

        size_t idx = keypoints_.size();
        keypoints_.push_back(std::move(kps));
        descriptors_.push_back(std::move(desc));
        sizes_.push_back(img.size());
        file_index_map_[relative] = idx;
    }
}

int PictureDB::index_of(const std::filesystem::path& relative_path) const {
    auto it = file_index_map_.find(relative_path);
    if (it == file_index_map_.end()) return -1;
    return static_cast<int>(it->second);
}
