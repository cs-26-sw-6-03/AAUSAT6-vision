#pragma once

#include <filesystem>
#include <map>
#include <opencv2/opencv.hpp>
#include <vector>

class PictureDB {
  public:
    explicit PictureDB(std::filesystem::path path_to_pictures);

    cv::Mat get_raw_frame(std::filesystem::path relative_path) const;

    void refresh(cv::Ptr<cv::ORB> orb);

    int index_of(const std::filesystem::path &relative_path) const;

    const std::vector<std::vector<cv::KeyPoint>> &keypoints() const { return keypoints_; }
    const std::vector<cv::Mat>                   &descriptors() const { return descriptors_; }
    const std::vector<cv::Size>                  &sizes() const { return sizes_; }

  private:
    std::filesystem::path pictures_path_;

    std::map<std::filesystem::path, size_t> file_index_map_;

    std::vector<std::vector<cv::KeyPoint>> keypoints_;
    std::vector<cv::Mat>                   descriptors_;
    std::vector<cv::Size>                  sizes_;
};
