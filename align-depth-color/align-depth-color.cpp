// align-depth-color.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include <librealsense2/rs.hpp>
#include "example.hpp"
#include <imgui.h>
#include "imgui_impl_glfw.h"

#include <algorithm>
#include <iterator>

#include <sstream>
#include <fstream>
#include <algorithm>
#include <cstring>
#include <iostream>

void render_slider( rect location, float& clipping_dist );
void remove_background( rs2::video_frame& other_frame, const rs2::depth_frame& depth_frame, float depth_scale, float clipping_dist );
void highlight_closest( rs2::video_frame& other_frame, const rs2::depth_frame& depth_frame, float depth_scale, float clipping_dist );
void array_to_csv( uint16_t* array, uint16_t length, const std::string& filename );
float get_depth_scale( rs2::device dev );
rs2_stream find_stream_to_align( const std::vector<rs2::stream_profile>& streams );
bool profile_changed( const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev );

int main( int argc, char* argv[] ) try {
    // Create and initialize GUI related objects.
    window app( 1280, 720, "Align" );		// Simple window handling.
    ImGui_ImplGlfw_Init( app, false );	// ImGui lib init.
    rs2::colorizer c;					// Helper to colorize depth image.
    texture renderer;					// Helper for rendering images.

    // Create a pipeline to config and init camera.
    rs2::pipeline pipe;
    // Start first device with its default stream.
    // The function returns the pipeline profile which the pipeline used to start the device.
    rs2::pipeline_profile profile = pipe.start();

    // Turn the laser off is possible.
    rs2::device selected_device = profile.get_device();
    auto depth_sensor = selected_device.first<rs2::depth_sensor>();

    if ( depth_sensor.supports( RS2_OPTION_EMITTER_ENABLED ) )
        depth_sensor.set_option( RS2_OPTION_EMITTER_ENABLED, 0.f );

    // Declare filters.
    rs2::decimation_filter dec_filter;

    // Configure filter parameters.
    dec_filter.set_option( RS2_OPTION_FILTER_MAGNITUDE, 3 );

    // Each depth camera might have different units for depth pixels, so we git it here.
    float depth_scale = get_depth_scale( profile.get_device() );

    // Pipeline could choose a device that does not have a color stream.
    // If there is no color stream, choose to align depth to another stream.
    rs2_stream align_to = find_stream_to_align( profile.get_streams() );

    // rs2::align allows to perform alignement of depth frames to other frames.
    // "align_to" is the stream type to which we plan to align depth frames.
    rs2::align align( align_to );

    // Define a variable for controlling the distance to clip.
    float depth_clipping_distance = 1.f;

    while ( app )	// Application still alive?
    {
        // Using the align object, we block the application until a framset is available.
        rs2::frameset frameset = pipe.wait_for_frames();

        // rs2::pipeline::wait_for_frames() can replace the device it uses in case of device error or disconnection.
        // Since rs2::align is aligning depth to some other stream, we need to make sure that the stream was not changed
        // after the call to wait_for_frames();
        if ( profile_changed( pipe.get_active_profile().get_streams(), profile.get_streams() ) ) {
            // If the profile was changed, update the align object, and also get the new device's depth scale.
            profile = pipe.get_active_profile();
            align_to = find_stream_to_align( profile.get_streams() );
            align = rs2::align( align_to );
            depth_scale = get_depth_scale( profile.get_device() );
        }

        // Get processed aligned frame.
        auto processed = align.process( frameset );

        // Trying to get both other and aligned depth frames.
        rs2::video_frame other_frame = processed.first( align_to );
        rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();

        // If one of them is unavailable, continue iteration.
        if ( !aligned_depth_frame || !other_frame ) {
            continue;
        }

        // Passing both frames to remove_background so it will "strip" the background.
        // NOTE: we alter the buffer of the other frame instead of copying and altering the copy.
        //		 This behavior is not recommened in real application since the other frame could be used elsewhere.
        remove_background( other_frame, aligned_depth_frame, depth_scale, depth_clipping_distance );
        //highlight_closest( other_frame, aligned_depth_frame, depth_scale, depth_clipping_distance );

        // Taking dimensions of the window for rendering purposes.
        float w = static_cast<float>(app.width());
        float h = static_cast<float>(app.height());

        // At this point, "other_frame" is an altered frame, stripped from its background.
        // Calculating the position to place the frame in the window.
        rect altered_other_frame_rect{ 0, 0, w, h };
        altered_other_frame_rect = altered_other_frame_rect.adjust_ratio( { static_cast<float>(other_frame.get_width()), static_cast<float>(other_frame.get_height()) } );

        // Render aligned image.
        renderer.render( other_frame, altered_other_frame_rect );

        // Renders the depth frame, as a picture-in-picture.
        // Calculating the postition to place the depth frame in the window.
        rect pip_stream{ 0, 0, w / 5, h / 5 };
        pip_stream = pip_stream.adjust_ratio( { static_cast<float>(aligned_depth_frame.get_width()), static_cast<float>(aligned_depth_frame.get_height()) } );
        pip_stream.x = altered_other_frame_rect.x + altered_other_frame_rect.w - pip_stream.w - (std::max( w, h ) / 25);
        pip_stream.y = altered_other_frame_rect.y + (std::max( w, h ) / 25);

        // Render depth (as picture in picture).
        renderer.upload( c.process( aligned_depth_frame ) );
        renderer.show( pip_stream );

        // Using ImGui lib to provide a slide controller to select the depth clipping distance.
        ImGui_ImplGlfw_NewFrame( 1 );
        render_slider( { 5.f, 0, w, h }, depth_clipping_distance );
        ImGui::Render();
    }
    return EXIT_SUCCESS;
}
catch ( const rs2::error & e ) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n\t" << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch ( const std::exception & e ) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

void render_slider( rect location, float& clipping_dist ) {
    // Some trickery to display the control nicely.
    static const int flags = ImGuiWindowFlags_NoCollapse
        | ImGuiWindowFlags_NoScrollbar
        | ImGuiWindowFlags_NoSavedSettings
        | ImGuiWindowFlags_NoTitleBar
        | ImGuiWindowFlags_NoResize
        | ImGuiWindowFlags_NoMove;
    const int pixels_to_buttom_of_stream_text = 25;
    const float slider_window_width = 30;

    ImGui::SetNextWindowPos( { location.x, location.y + pixels_to_buttom_of_stream_text } );
    ImGui::SetNextWindowSize( { slider_window_width + 20, location.h - (pixels_to_buttom_of_stream_text * 2) } );

    // Render the vertical slider.
    ImGui::Begin( "slider", nullptr, flags );
    ImGui::PushStyleColor( ImGuiCol_FrameBg, ImColor( 215.f / 255, 215.0f / 255, 215.0f / 255 ) );
    ImGui::PushStyleColor( ImGuiCol_SliderGrab, ImColor( 215.f / 255, 215.0f / 255, 215.0f / 255 ) );
    ImGui::PushStyleColor( ImGuiCol_SliderGrabActive, ImColor( 215.f / 255, 215.0f / 255, 215.0f / 255 ) );
    auto slider_size = ImVec2( slider_window_width / 2, location.h - (pixels_to_buttom_of_stream_text * 2) - 20 );
    ImGui::VSliderFloat( "", slider_size, &clipping_dist, 0.0f, 6.0f, "", 1.0f, true );
    if ( ImGui::IsItemHovered() )
        ImGui::SetTooltip( "Depth Clipping Distance: %.3f", clipping_dist );
    ImGui::PopStyleColor( 3 );

    // Display bar next to slider.
    float bars_dist = (slider_size.y / 6.0f);
    for ( int i = 0; i <= 6; i++ ) {
        ImGui::SetCursorPos( { slider_size.x, i * bars_dist } );
        std::string bar_text = "- " + std::to_string( 6 - i ) + "m";
        ImGui::Text( "%s", bar_text.c_str() );
    }
    ImGui::End();
}

void remove_background( rs2::video_frame & other_frame, const rs2::depth_frame & depth_frame, float depth_scale, float clipping_dist ) {
    const uint16_t* p_depth_frame = reinterpret_cast<const uint16_t*>(depth_frame.get_data());
    uint8_t* p_other_frame = reinterpret_cast<uint8_t*>(const_cast<void*>(other_frame.get_data()));

    int width = other_frame.get_width();
    int height = other_frame.get_height();
    int other_bpp = other_frame.get_bytes_per_pixel();

#pragma omp parallel for schedule(dynamic)  // Using OpenMP to try to parallelise the loop.
    for ( int y = 0; y < height; y++ ) {
        auto depth_pixel_index = y * width;
        for ( int x = 0; x < width; x++, ++depth_pixel_index ) {
            // Get the depth value of the current pixel.
            auto pixels_distance = depth_scale * p_depth_frame[depth_pixel_index];

            // Check if the depth value is invalid (<=0) or greater than the treashold.
            if ( pixels_distance <= 0.f || pixels_distance > clipping_dist ) {
                // Calculate the offset in other frame's buffer to current pixel.
                auto offset = depth_pixel_index * other_bpp;

                // Set pixel to "background" color (0x999999).
                std::memset( &p_other_frame[offset], 0x99, other_bpp );
            }
        }
    }
}

void highlight_closest( rs2::video_frame & other_frame, const rs2::depth_frame & depth_frame, float depth_scale, float clipping_dist ) {
    const int slotSizeFactor = 5;
    const int noOfSlots = 65536 >> slotSizeFactor;
    uint16_t slot_counts[noOfSlots];

    // Clear the depth counters in each slot.
    for ( int i = 0; i < sizeof( slot_counts ) / sizeof( *slot_counts ); i++ )
        slot_counts[i] = 0;

    uint16_t * p_depth_frame = reinterpret_cast<uint16_t*>(const_cast<void*>(depth_frame.get_data()));
    uint8_t * p_other_frame = reinterpret_cast<uint8_t*>(const_cast<void*>(other_frame.get_data()));

    const int width = other_frame.get_width();
    const int height = other_frame.get_height();
    int other_bpp = other_frame.get_bytes_per_pixel();

#pragma omp parallel for schedule(dynamic)  // Using OpenMP to try to parallelise the loop.
    // Make a pass through the image counting depth pixels.
    for ( int y = 0; y < height; y++ ) {
        auto depth_pixel_index = y * width;
        for ( int x = 0; x < width; x++, ++depth_pixel_index ) {
            // Get the depth value of the current pixel.
            auto pixels_distance = p_depth_frame[depth_pixel_index];

            // If invalid value
            if ( pixels_distance * depth_scale <= 0.f || pixels_distance * depth_scale > clipping_dist )
                continue;

            // Find the slot and increase the counter.
            slot_counts[pixels_distance >> slotSizeFactor]++;
        }
    }

    // Now find the depth with the most pixels.
    int max_count = 0;
    int max_pos = 0;

#pragma omp parallel for schedule(dynamic)
    for ( int i = 0; i < sizeof( slot_counts ) / sizeof( *slot_counts ); i++ ) {
        if ( slot_counts[i] > max_count ) {
            max_count = slot_counts[i];
            max_pos = i;
        }
    }

//#pragma omp parallel for schedule(dynamic)  // Using OpenMP to try to parallelise the loop.
//    // Make a pass through the image counting depth pixels.
//    for ( int y = 0; y < height; y++ ) {
//        auto depth_pixel_index = y * width;
//        auto from_x = (width * height) + 1;
//        for ( int x = 0; x < width; x++, ++depth_pixel_index ) {
//            // Get the depth value of the current pixel.
//            auto pixels_distance = p_depth_frame[depth_pixel_index];
//
//            if ( pixels_distance >> slotSizeFactor == max_pos ) {
//                if ( p_depth_frame[depth_pixel_index + 1] <= 0 ) {
//                    from_x = depth_pixel_index;
//                }
//                if ( p_depth_frame[depth_pixel_index - 1] <= 0 && from_x < ((width * height) + 1) ) {
//                    if ( depth_pixel_index - from_x > 50 )
//                        continue;
//                    for ( auto i = from_x + 1; i < depth_pixel_index; i++ ) {
//                        p_depth_frame[i] = p_depth_frame[from_x];
//                    }
//                    auto from_x = (width * height) + 1;
//                }
//            }
//        }
//    }

    uint32_t x_total = 0;
    uint32_t y_total = 0;
    // Now Remove the background
#pragma omp parallel for schedule(dynamic)  // Using OpenMP to try to parallelise the loop.
    for ( int y = 0; y < height; y++ ) {
        auto depth_pixel_index = y * width;
        for ( int x = 0; x < width; x++, ++depth_pixel_index ) {
            // Get the depth value of the current pixel.
            auto pixels_distance = p_depth_frame[depth_pixel_index];
            // Calculate the offset in other frame's buffer to current pixel.
            auto offset = depth_pixel_index * other_bpp;
            if ( pixels_distance >> slotSizeFactor == max_pos ) {
                x_total += x;
                y_total += y;
                //std::memset( &p_other_frame[offset], 0x00, other_bpp );
            }
            else {
                // Remove background
                p_other_frame[offset] = 0x00;   // R
                p_other_frame[offset+1] = 0x00; // G
                p_other_frame[offset+2] = 0x00; // B
            }
        }
    }

//    uint32_t x;
//    uint32_t y;
//
//    if ( slot_counts[max_pos] == 0 ) {
//        x = width / 2;
//        y = height / 2;
//    }
//    else {
//        x = x_total / slot_counts[max_pos];
//        y = y_total / slot_counts[max_pos];
//    }
//
//    if ( x + 10 > width ) x = width - 10;
//    else if ( x - 10 > width )x = 10;   // If x-10 would be < 0.
//    if ( y + 10 > height ) y = height - 10;
//    else if ( y - 10 > height )y = 10;   // If y-10 would be < 0.
//
//    std::cout << "Biggest object at (" << x << ", " << y << ")     \r";
//#pragma omp parallel for schedule(dynamic)  // Using OpenMP to try to parallelise the loop.
//    for ( auto iy = y - 10; iy <= y + 10; iy++ ) {
//        for ( auto ix = x - 1; ix <= x + 1; ix++ ) {
//            auto depth_pixel_index = (iy * width) + ix;
//            std::memset( &p_other_frame[depth_pixel_index * other_bpp], 0xFF, other_bpp );
//        }
//    }
//#pragma omp parallel for schedule(dynamic)  // Using OpenMP to try to parallelise the loop.
//    for ( auto iy = y - 1; iy <= y + 1; iy++ ) {
//        for ( auto ix = x - 10; ix <= x + 10; ix++ ) {
//            auto depth_pixel_index = (iy * width) + ix;
//            std::memset( &p_other_frame[depth_pixel_index * other_bpp], 0xFF, other_bpp );
//        }
//    }
}

void array_to_csv( uint16_t * array, uint16_t length, const std::string & filename ) {
    std::ofstream csv;
    csv.open( filename );
    for ( uint16_t i = 0; i < length; i++ ) {
        csv << array[i] << "\n";
    }
    csv.close();
}

float get_depth_scale( rs2::device dev ) {
    // Go over the device's sensors.
    for ( rs2::sensor& sensor : dev.query_sensors() ) {
        // Check if the sensor is a depth sensor.
        if ( rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>() ) {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error( "Device does not have a depth sensor" );
}

rs2_stream find_stream_to_align( const std::vector<rs2::stream_profile> & streams ) {
    // Given a vector of streams, we try to find a depth stream and another stream to align depth with.
    // We prioritize color streams to make the view look better.
    // If color is not available, we take another stream that isn't the depth stream
    rs2_stream align_to = RS2_STREAM_ANY;
    bool depth_stream_found = false;
    bool color_stream_found = false;
    for ( rs2::stream_profile sp : streams ) {
        rs2_stream profile_stream = sp.stream_type();
        if ( profile_stream != RS2_STREAM_DEPTH ) {
            if ( !color_stream_found )     // Prefer color.
                align_to = profile_stream;

            if ( profile_stream == RS2_STREAM_COLOR )
                color_stream_found = true;
        }
        else
            depth_stream_found = true;
    }

    if ( !depth_stream_found )
        throw std::runtime_error( "No Depth stream available" );

    if ( align_to == RS2_STREAM_ANY )
        throw std::runtime_error( "No stream found to align with Depth" );

    return align_to;
}

bool profile_changed( const std::vector<rs2::stream_profile> & current, const std::vector<rs2::stream_profile> & prev ) {
    for ( auto&& sp : prev ) {
        // If previous profile is in current profile (maybe another profile was added).
        auto itr = std::find_if( std::begin( current ), std::end( current ), [&sp]( const rs2::stream_profile & current_sp ) {return sp.unique_id() == current_sp.unique_id(); } );
        if ( itr == std::end( current ) )  // If the previous profile wasn't found in current profile.
            return true;
    }
    return false;
}
