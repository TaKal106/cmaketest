#include <iostream>
#include <stack>
#include <string>
#include <dirent.h> // for Linux
#include <sys/stat.h> // for Linux
// #include <opencv2/opencv.hpp>

#include "camera_calib.h"

std::vector<std::string> get_all_files_names_within_folder(std::string folder) {
  std::vector<std::string> names;
  DIR* dir;
  struct dirent* ent;
  struct stat st;
  dir = opendir(folder.c_str());
  while ((ent = readdir(dir)) != NULL) {
    const std::string file_name = ent->d_name;
    const std::string full_file_name = folder + "/" + file_name;

    if (file_name[0] == '.')
      continue;

    if (stat(full_file_name.c_str(), &st) == -1)
      continue;

    if ((st.st_mode & S_IFMT) == S_IFDIR)
      continue;

    names.push_back(file_name);
  }
  closedir(dir);
  return names;
}

int main(int argc, char** argv) {
  std::string images_dir = "ori/";
  std::vector<std::string> image_names;
  image_names = get_all_files_names_within_folder(images_dir);
  for (int i = 0; i < image_names.size(); ++i)
    std::cout << image_names[i] << std::endl;
#if 0
  FisheyeCameraCalib(images_dir, image_names);
#endif
#if 1
  UndistortImages(images_dir, image_names, true);
#endif
#if 0
  std::string root_dir = "Release/";
  std::vector<int> camera_idx_set = { 0 };
  for (int i = 0; i < camera_idx_set.size(); ++i) {
    int camera_idx = camera_idx_set[i];
    std::string images_dir = root_dir + "camera_calib_" + std::to_string(camera_idx);
    TraditionCameraCalib(images_dir, camera_idx);
  }
#endif
  return 0;
}
