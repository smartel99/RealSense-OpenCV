// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include <fstream>
#include <iostream>
#include <sstream>

// 3rd party header for writing png files
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include "example.hpp"          // Include short list of convenience functions for rendering

// Helper function for writing metadata to disk as a csv file
void metadata_to_csv(const rs2::frame& frm, const std::string& filename);

int main(int argc, char* argv[]) try
{
	rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
	// Create a simple OpenGL window for rendering:
	window app(1280, 720, "RealSense Capture Example");

	// Declare depth colorizer for pretty visualization of depth data
	rs2::colorizer color_map;
	// Declare rates printer for showing streaming rates of the enabled streams.
	rs2::rates_printer printer;

	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe;

	// Start streaming with default recommended configuration
	// The default video configuration contains Depth and Color streams
	pipe.start();

	// Capture 30 frames to give autoexposure a chance to settle.
	for (auto i=0; i < 30; i++) pipe.wait_for_frames();

	// Wait for the next set of frames from the camera, which will be saved to the disk.
	for (auto&& frame : pipe.wait_for_frames()) {
		// Video frames can only be saved as pngs.
		if (auto vf = frame.as<rs2::video_frame>()) {
			auto stream = frame.get_profile().stream_type();
			// Use the colorizer to get an rgb image for the depth stream.
			if (vf.is<rs2::depth_frame>()) vf = color_map.process(frame);

			// Write images to disk
			std::stringstream png_file;
			png_file << "rs-save-to-disk-output-" << vf.get_profile().stream_name() << ".png";
			stbi_write_png(png_file.str().c_str(), vf.get_width(), vf.get_height(),
				vf.get_bytes_per_pixel(), vf.get_data(), vf.get_stride_in_bytes());
			std::cout << "Saved " << png_file.str() << std::endl;

			// Record per-frame metadata 
			std::stringstream csv_file;
			csv_file << "rs-save-to-disk-output-" << vf.get_profile().stream_name() << "-metadata.csv";
			metadata_to_csv(vf, csv_file.str());
		}
	}

	while (app) // Application still alive?
	{
		rs2::frameset data = pipe.wait_for_frames().    // Wait for next set of frames from the camera
			apply_filter(printer).     // Print each enabled stream frame rate
			apply_filter(color_map);   // Find and colorize the depth data

// The show method, when applied on frameset, break it to frames and upload each frame into a gl textures
// Each texture is displayed on different viewport according to it's stream unique id
		app.show(data);
	}

	return EXIT_SUCCESS;
}
catch (const rs2::error& e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception& e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}

void metadata_to_csv(const rs2::frame& frm, const std::string& filename)
{
	std::ofstream csv;

	csv.open(filename);

	std::cout << "Writting metadata to " << filename << std::endl;
	csv << "Stream," << rs2_stream_to_string(frm.get_profile().stream_type()) << "\nMetadata Attribute,Value\n";
	// Record all the available metadata attributes.
	for (size_t i = 0; i < RS2_FRAME_METADATA_COUNT; i ++ ) {
		if (frm.supports_frame_metadata((rs2_frame_metadata_value)i)) {
			csv << rs2_frame_metadata_to_string((rs2_frame_metadata_value)i) << ","
				<< frm.get_frame_metadata((rs2_frame_metadata_value)i) << "\n";
		}
	}

	csv.close();
}
