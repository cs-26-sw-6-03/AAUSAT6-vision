#pragma once

#include <filesystem>
#include <map>
#include <opencv2/opencv.hpp>
#include <vector>

class PictureDB {
  public:
    explicit PictureDB(std::filesystem::path path_to_pictures);

    // Returns the raw image for a file relative to the pictures directory.
    cv::Mat get_raw_frame(std::filesystem::path relative_path) const;

    // Scans the pictures directory for files not yet loaded.
    // Computes keypoints and descriptors only for new files using the provided ORB model.
    // Already-loaded files are left untouched.
    void refresh(cv::Ptr<cv::ORB> orb);

    // Returns the index of a loaded file in the keypoints/descriptors vectors.
    // Returns -1 if the file has not been loaded.
    int index_of(const std::filesystem::path &relative_path) const;

    const std::vector<std::vector<cv::KeyPoint>> &keypoints() const { return keypoints_; }
    const std::vector<cv::Mat>                   &descriptors() const { return descriptors_; }
    const std::vector<cv::Size>                  &sizes() const { return sizes_; }

  private:
    std::filesystem::path pictures_path_;

    // Maps each loaded file's path (relative to pictures_path_) to its index
    // in the keypoints_ and descriptors_ vectors.
    std::map<std::filesystem::path, size_t> file_index_map_;

    std::vector<std::vector<cv::KeyPoint>> keypoints_;
    std::vector<cv::Mat>                   descriptors_;
    std::vector<cv::Size>                  sizes_;
};
