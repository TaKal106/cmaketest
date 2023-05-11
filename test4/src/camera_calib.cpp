#include <iostream>
// #include <direct.h>
#include <unistd.h>
#include <stack>
#include <opencv2/imgcodecs/legacy/constants_c.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/core/types_c.h>
#include <string>
#include <iostream>
#include <stdio.h>
#include <cassert>
// #include <opencv2/viz.hpp>

void FisheyeCameraCalib(std::string images_dir, std::vector<std::string> image_names) {

	int pattern_rows = 7;
	int pattern_cols = 10;
	cv::Size patternsize(pattern_cols, pattern_rows); //interior number of corners
	std::vector<cv::Point3d> objp;
	for (int row = 0; row < patternsize.height; ++row) {
		for (int col = 0; col < patternsize.width; ++col) {
			objp.emplace_back(cv::Point3d(row, col, 0));
		}
	}

	std::vector<std::vector<cv::Point3d> > objpoints;
	std::vector<std::vector<cv::Point2f> > imgpoints;
	cv::Size image_size;
	for (int i = 0; image_names.size(); ++i) {

		char name[512];
		sprintf(name, (images_dir + image_names[i]).c_str(), i);
		cv::Mat color_image = cv::imread(name);
		if (color_image.rows == 0) break;
		cv::Mat gray_image;
		cv::cvtColor(color_image, gray_image, cv::COLOR_BGR2GRAY);//source image
		if (i == 0) image_size = color_image.size();
		std::vector<cv::Point2f> corners; //this will be filled by the detected corners

		//CALIB_CB_FAST_CHECK saves a lot of time on images
		//that do not contain any chessboard corners
		bool patternfound = findChessboardCorners(gray_image, patternsize, corners,
																							cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
																							+ cv::CALIB_CB_FAST_CHECK);

		if (patternfound) {
			objpoints.emplace_back(objp);
			cv::cornerSubPix(gray_image, corners, cv::Size(3, 3), cv::Size(-1, -1),
											 cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			imgpoints.emplace_back(corners);

			cv::drawChessboardCorners(color_image, patternsize, cv::Mat(corners), patternfound);
		}

		cv::imshow("color_image", color_image);
		cv::waitKey(1);
	}

	cv::Mat Ka = cv::Mat::eye(3, 3, CV_64F); // Creating distortion matrix
	cv::Mat Da = cv::Mat::ones(1, 4, CV_64F);
	cv::Mat Knew = cv::Mat::eye(3, 3, CV_64F);
	std::vector<cv::Vec3d> rvec;
	std::vector<cv::Vec3d> tvec;

	//int calibration_flags = cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC + cv::fisheye::CALIB_CHECK_COND + cv::fisheye::CALIB_FIX_SKEW;
	int calibration_flags = cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC + cv::fisheye::CALIB_CHECK_COND + cv::fisheye::CALIB_FIX_SKEW;
	cv::fisheye::calibrate(objpoints, imgpoints, image_size, Ka, Da, rvec, tvec, calibration_flags); // Calibration
	std::cout << "K matrix: " << Ka << std::endl;
	std::cout << "D matrix: " << Da << std::endl;
	cv::FileStorage fs(images_dir + "/../camera_intrin.yaml", cv::FileStorage::WRITE);
	fs << "K " << Ka;
	fs << "D " << Da;
	fs << "image_size " << image_size;
	fs.release();

	bool vis = true;
	if (vis) {
		cv::Mat eye_mat = cv::Mat::eye(3, 3, CV_32F);
		cv::Mat map1, map2;
		cv::fisheye::initUndistortRectifyMap(Ka, Da, eye_mat, Ka, image_size, CV_16SC2, map1, map2);
		cv::Mat rec_color_image;
		for (int i = 0; ; ++i) {

			char name[512];
			sprintf(name, (images_dir + image_names[i]).c_str(), i);
			cv::Mat color_image = cv::imread(name);
			if (color_image.rows == 0) break;
			cv::remap(color_image, rec_color_image, map1, map2, cv::INTER_LINEAR);
			cv::imshow("rec_color_image", rec_color_image);
			cv::waitKey(0);
		}
	}
}

void UndistortImages(std::string images_dir, std::vector<std::string> image_names, bool is_fisheye) {

	cv::Mat Ka = cv::Mat::eye(3, 3, CV_64F); // Creating distortion matrix
	cv::Mat Da = cv::Mat::ones(1, 4, CV_64F);
	cv::Size image_size;
	cv::FileStorage fs(images_dir + "/../camera_intrin.yaml", cv::FileStorage::READ);
	fs["K"] >> Ka;
	fs["D"] >> Da;
	fs["image_size"] >> image_size;
	fs.release();
	std::cout << Ka << std::endl;
	std::cout << Da << std::endl;
	std::cout << image_size << std::endl;

	cv::Mat eye_mat = cv::Mat::eye(3, 3, CV_32F);
	cv::Mat map1, map2;

	cv::Mat new_Ka;
	if (is_fisheye) {
		image_size = cv::Size(image_size.width * 1.0, image_size.height * 1.0);
		cv::fisheye::estimateNewCameraMatrixForUndistortRectify(Ka, Da, image_size, eye_mat,
			new_Ka, 0.3, image_size);
		cv::fisheye::initUndistortRectifyMap(Ka, Da, eye_mat, new_Ka, image_size, CV_16SC2, map1, map2);
	}
	else {
		new_Ka = Ka;
		cv::initUndistortRectifyMap(Ka, Da, eye_mat, new_Ka, image_size, CV_16SC2, map1, map2);
	}
	std::cout << new_Ka << std::endl;
	fs = cv::FileStorage(images_dir + "/../camera_intrin_undistort.yaml", cv::FileStorage::WRITE);
	fs << "K " << new_Ka;
	fs << "D " << Da;
	fs << "image_size " << image_size;
	fs.release();

	for (int i = 0; image_names.size(); ++i) {

		char name[512];
		sprintf(name, (images_dir + image_names[i]).c_str(), i);
		cv::Mat color_image = cv::imread(name);

		cv::Mat rec_color_image;
		int count = 0;
		cv::remap(color_image, rec_color_image, map1, map2, cv::INTER_LINEAR);
		cv::imshow("color_image", color_image);
		cv::rotate(rec_color_image, rec_color_image, 0);
		cv::imshow("rec_color_image", rec_color_image);
		char path[256];
		sprintf(path, (images_dir + "/../undistort/%06d_undist.png").c_str(), i);
		cv::imwrite(path, rec_color_image);
		cv::waitKey(1);
		if (count == 0) {
			cv::Mat mask_image = cv::Mat::ones(color_image.size(), CV_8UC1) * 255;
			cv::remap(mask_image, mask_image, map1, map2, cv::INTER_LINEAR);
			cv::imwrite(images_dir + "/../mask.png", mask_image);
		}
		std::cout << count++ << "\r";
	}
}

void TraditionCameraCalib(std::string images_dir, int camera_idx) {

	int pattern_rows = 6;
	int pattern_cols = 9;
	cv::Size patternsize(pattern_cols, pattern_rows); //interior number of corners
	std::vector<cv::Point3f> objp;
	for (int row = 0; row < patternsize.height; ++row) {
		for (int col = 0; col < patternsize.width; ++col) {
			objp.emplace_back(cv::Point3d(row, col, 0));
		}
	}

	std::vector<std::vector<cv::Point3f> > objpoints;
	std::vector<std::vector<cv::Point2f> > imgpoints;
	cv::Size image_size;
	for (int i = 0; ; ++i) {
		std::cout << i << std::endl;

		char name[512];
		sprintf(name, (images_dir + "/%05d_color.png").c_str(), i);
		cv::Mat color_image = cv::imread(name);
		if (color_image.rows == 0) break;
		cv::Mat gray_image;
		cv::cvtColor(color_image, gray_image, cv::COLOR_BGR2GRAY);//source image
		if (i == 0) image_size = color_image.size();
		std::vector<cv::Point2f> corners; //this will be filled by the detected corners

		//CALIB_CB_FAST_CHECK saves a lot of time on images
		//that do not contain any chessboard corners
		bool patternfound = findChessboardCorners(gray_image, patternsize, corners,
																							cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
																							+ cv::CALIB_CB_FAST_CHECK);

		if (patternfound) {
			objpoints.emplace_back(objp);
			cv::cornerSubPix(gray_image, corners, cv::Size(3, 3), cv::Size(-1, -1),
											 cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			imgpoints.emplace_back(corners);

			cv::drawChessboardCorners(color_image, patternsize, cv::Mat(corners), patternfound);
		}

		cv::Mat temp_color_image = color_image;
		//cv::resize(color_image, temp_color_image, cv::Size(color_image.cols / 4, color_image.rows / 4));
		cv::imshow("temp_color_image", temp_color_image);
		cv::waitKey(1);
	}

	cv::Mat Ka; // Creating distortion matrix
	std::vector<cv::Mat> rvec;
	std::vector<cv::Mat> tvec;
	cv::Mat dist_coeffs;

	cv::calibrateCamera(objpoints, imgpoints, image_size, Ka, dist_coeffs,  rvec, tvec); // Calibration
	std::cout << "K matrix: " << Ka << std::endl;
	std::cout << "D: " << dist_coeffs << std::endl;
	cv::FileStorage fs(images_dir + "/../camera_" + std::to_string(camera_idx) + "_intrin.yaml", cv::FileStorage::WRITE);
	fs << "K " << Ka;
	fs << "D " << dist_coeffs;
	fs << "image_size " << image_size;
	fs.release();

	bool vis = false;
	if (vis) {
		cv::Mat eye_mat = cv::Mat::eye(3, 3, CV_32F);
		cv::Mat map1, map2;
		cv::initUndistortRectifyMap(Ka, dist_coeffs, eye_mat, Ka, image_size, CV_16SC2, map1, map2);
		cv::Mat rec_color_image;
		for (int i = 0; ; ++i) {

			char name[512];
			sprintf(name, (images_dir + "/05d_color.png").c_str(), i);
			cv::Mat color_image = cv::imread(name);
			if (color_image.rows == 0) break;
			cv::remap(color_image, rec_color_image, map1, map2, cv::INTER_LINEAR);
			cv::imshow("rec_color_image", rec_color_image);
			cv::waitKey(0);
		}
	}
}

void UndistortVideo(std::string video_path, int camera_idx, bool is_fisheye) {

	cv::Mat Ka = cv::Mat::eye(3, 3, CV_64F); // Creating distortion matrix
	cv::Mat Da = cv::Mat::ones(1, 4, CV_64F);
	cv::Size image_size;
	cv::FileStorage fs(video_path + "/../camera_" + std::to_string(camera_idx) + "_intrin.yaml", cv::FileStorage::READ);
	fs["K"] >> Ka;
	fs["D"] >> Da;
	fs["image_size"] >> image_size;
	fs.release();
	std::cout << Ka << std::endl;
	std::cout << Da << std::endl;
	std::cout << image_size << std::endl;

	cv::Mat eye_mat = cv::Mat::eye(3, 3, CV_32F);
	cv::Mat map1, map2;

	cv::Mat new_Ka;
	cv::Size old_image_size = image_size;
	if (is_fisheye) {
    //image_size = cv::Size(image_size.width * 1.0, image_size.height * 1.0);
    image_size = cv::Size(image_size.width * 1.5, image_size.height * 1.5);
		cv::fisheye::estimateNewCameraMatrixForUndistortRectify(Ka, Da, old_image_size, eye_mat,
																														new_Ka, 0.6, image_size);
		cv::fisheye::initUndistortRectifyMap(Ka, Da, eye_mat, new_Ka, image_size, CV_16SC2, map1, map2);
	} else {
		new_Ka = Ka;
		cv::initUndistortRectifyMap(Ka, Da, eye_mat, new_Ka, image_size, CV_16SC2, map1, map2);
	}
	std::cout << new_Ka << std::endl;
	fs = cv::FileStorage(video_path + "/../camera_" + std::to_string(camera_idx) + "_intrin_undistort.yaml", cv::FileStorage::WRITE);
	fs << "K " << new_Ka;
	fs << "D " << Da;
	fs << "image_size " << image_size;
	fs.release();

	cv::VideoWriter output_video;                                        // Open the output
	output_video.open(video_path + "/../" + std::to_string(camera_idx) + "_undistort.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, image_size, true);

	cv::VideoCapture color_video_capture;
	color_video_capture.open(video_path);
	cv::Mat color_image, mask_image;
	cv::Mat rec_color_image;
	int count = 0;
	while (color_video_capture.read(color_image)) {
		cv::remap(color_image, rec_color_image, map1, map2, cv::INTER_LINEAR);
		//cv::imshow("rec_color_image", rec_color_image);
		//cv::waitKey(1);
		output_video << rec_color_image;
		if (count == 0) {
			mask_image = cv::Mat::ones(color_image.size(), CV_8UC1) * 255;
			cv::remap(mask_image, mask_image, map1, map2, cv::INTER_LINEAR);
			cv::imwrite(video_path + "/../" + std::to_string(camera_idx) + "_mask.png", mask_image);
		}
		std::cout << count++ << "\r";
	}
	color_video_capture.release();
	output_video.release();
}

