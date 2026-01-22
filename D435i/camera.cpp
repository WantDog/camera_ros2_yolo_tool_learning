#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>

int main()
{
    try
    {
        rs2::pipeline pipe;
        rs2::config cfg;

        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        pipe.start(cfg);

        rs2::frameset frames = pipe.wait_for_frames();
        rs2::video_frame color_frame = frames.get_color_frame();
        int width = color_frame.get_width();
        int height = color_frame.get_height();

        cv::VideoWriter writer;
        bool is_recording = false;
        int video_index = 0;

        std::cout << "按 'r' 开始录制，按 's' 停止录制，按 'ESC' 退出程序。" << std::endl;

        while (true)
        {
            frames = pipe.wait_for_frames();
            color_frame = frames.get_color_frame();

            cv::Mat color(cv::Size(width, height), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
            cv::imshow("RealSense Color", color);

            if (is_recording && writer.isOpened())
                writer.write(color);

            int key = cv::waitKey(1);
            if (key == 27) // ESC键退出
                break;
            else if (key == 'r' || key == 'R')
            {
                if (!is_recording)
                {
                    std::ostringstream filename;
                    filename << "recorded_" << video_index++ << ".mp4";

                    int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
                    double fps = 30.0;
                    cv::Size frame_size(width, height);
                    writer.open(filename.str(), codec, fps, frame_size, true);

                    if (!writer.isOpened())
                    {
                        std::cerr << "无法打开文件进行写入！" << std::endl;
                        return EXIT_FAILURE;
                    }

                    is_recording = true;
                    std::cout << "[开始录制] -> " << filename.str() << std::endl;
                }
            }
            else if (key == 's' || key == 'S')
            {
                if (is_recording)
                {
                    writer.release();
                    is_recording = false;
                    std::cout << "[停止录制]" << std::endl;
                }
            }
        }

        if (writer.isOpened())
            writer.release();

        pipe.stop();
    }
    catch (const rs2::error &e)
    {
        std::cerr << "RealSense 错误: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    catch (const std::exception &e)
    {
        std::cerr << "其他错误: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
