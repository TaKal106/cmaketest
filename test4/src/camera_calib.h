#pragma once

#ifndef _MAINWINDOW_H_
#define _MAINWINDOW_H_
#if defined(_WIN32)
#pragma execution_character_set("utf-8")
#endif

#include <string>
#include <vector>

void FisheyeCameraCalib(std::string images_dir, std::vector<std::string> image_names);
void UndistortImages(std::string images_dir, std::vector<std::string> image_names, bool is_fisheye);
void TraditionCameraCalib(std::string images_dir, int camera_idx);

#endif // _MAINWINDOW_H_

