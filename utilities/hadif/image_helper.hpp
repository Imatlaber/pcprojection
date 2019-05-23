/* Copyright (c) AutoNavi - All Rights Reserved
 * Author: shuo <shuo@alibaba-inc.com>
 * description: image helper
 */

#ifndef _IMAGE_HELPER_
#define _IMAGE_HELPER_

#include <opencv2/core/core.hpp>
#include <png++/gray_pixel.hpp>
#include <png++/image.hpp>
#include <turbojpeg.h>
#include <iostream>
#include <snappy.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <iterator>
#include <opencv2/imgproc/imgproc.hpp>
#include <turbojpeg.h>
#include <snappy.h>
#include <png++/image.hpp>
#include "lcmtypes/had/image_t.hpp"
#include "../../utilities/hadif/time_helper.hpp"

class ImageHelper
{
    had::image_t imageMsg_ ;
    std::string unCompressed_ ;
    uint8_t* temp_ ;
    uint8_t* bgr8Buffer_ ;
    bool isPrintStatus_ ;

    public:

    // ---------- encoding -----------
    static void packageUint8AsMessage(const uint8_t* buffer, const size_t bufferSize,
                                              const int32_t height, const int32_t width,
                                              const uint8_t imageCompression, const uint8_t imageType,
                                              const had::header_t imageHeader, had::image_t& imageMsg, bool isPrintStatus=false)
    {
        Stopwatch sw;
    
        imageMsg.height = height;
        imageMsg.width = width;
        imageMsg.data = std::vector< uint8_t >(buffer, buffer + bufferSize);
        imageMsg.size = static_cast<int32_t>(imageMsg.data.size());
        imageMsg.compression.enumerate = static_cast<int8_t>(imageCompression);
        imageMsg.type.enumerate = static_cast<int8_t>(imageType);
        imageMsg.header = imageHeader;
    
        if(isPrintStatus)
            printf ("t_pack=%06.3f, ", sw.toc() * 1000); // display milli seconds
    }
    
    static bool packageMatBgrAsJpegMessage(const cv::Mat& mat, const int jpegQual, const had::header_t imageHeader, had::image_t& imageMsg, bool isPrintStatus=false)
    {
        Stopwatch sw;
    
        tjhandle tjCompressHandler = tjInitCompress();
    
        ulong jpegBufferSize {0};
        uint8_t* jpegBuffer;
        int statusInt = tjCompress2(tjCompressHandler, mat.data,
                                    mat.cols, 0, mat.rows, TJPF_BGR,
                                    &jpegBuffer, &jpegBufferSize,
                                    TJSAMP_422, jpegQual, TJFLAG_FASTDCT);
    
        if(statusInt < 0) { return false; }
    
        if(isPrintStatus)
            printf ("t_jpeg=%06.3f, ", sw.toc() * 1000);
    
        packageUint8AsMessage(jpegBuffer, jpegBufferSize, mat.rows, mat.cols,
                              had::image_compression_t::NONE, had::image_type_t::JPEG,
                              imageHeader, imageMsg);
    
        tjDestroy(tjCompressHandler);
        tjFree(jpegBuffer);
    
        if(isPrintStatus)
            printf ("jpegSize=%.3fMB, ", imageMsg.size / (1024.0 * 1024.0));
    
    
        return true;
    }
    
    
    static bool packageMatBgrAsUintBgr(const cv::Mat& mat, const had::header_t imageHeader, had::image_t& imageMsg, bool isPrintStatus=false)
    {
        Stopwatch sw;
    
        packageUint8AsMessage(mat.ptr(), mat.rows*mat.cols*3 , mat.rows, mat.cols,
                              had::image_compression_t::NONE, had::image_type_t::BGR8,
                              imageHeader, imageMsg);
    
        if(isPrintStatus)
            printf ("jpegSize=%.3fMB, ", imageMsg.size / (1024.0 * 1024.0));
    
    
        return true;
    }
    
    static bool packageMatMono8AsMessage(const cv::Mat& mat, const had::header_t imageHeader, had::image_t& imageMsg, bool isPrintStatus=false)
    {
        Stopwatch sw;
    
        packageUint8AsMessage(mat.ptr(), mat.rows*mat.cols , mat.rows, mat.cols,
                              had::image_compression_t::NONE, had::image_type_t::MONO8,
                              imageHeader, imageMsg);
    
        if(isPrintStatus)
            printf ("MonoSize=%.3fMB, ", imageMsg.size / (1024.0 * 1024.0));
    
    
        return true;
    }
    
    static bool packageYUV422AsMatBgr(const uint8_t* rawData,
            const int32_t height, const int32_t width,
            cv::Mat& mat, bool /* is_print_status = false*/) 
    {
        mat = cv::Mat(height, width, CV_8UC2, const_cast<uint8_t*>(rawData));
        cv::cvtColor(mat, mat, cv::COLOR_YUV2BGR_Y422);
    
            return true;
    }
    
    // (TODO) test others for Y411?
    // COLOR_YUV2BGR_NV12
    // COLOR_YUV2BGR_NV21
    // COLOR_YUV2BGR_YV12
    // COLOR_YUV2BGR_IYUV (test this first)
    static bool packageYUV411AsMatBgr(const uint8_t* rawData,
            const int32_t height, const int32_t width, cv::Mat& mat)
    {
        mat = cv::Mat(height, width, CV_8UC2, const_cast<uint8_t*>(rawData));
        cv::cvtColor(mat, mat, cv::COLOR_YUV2BGR_IYUV);
    
        return true;
    }
    
    
    static void packageYUV422AsSnappyMessage(const uint8_t* buffer, const size_t bufferSize, const int32_t height, const int32_t width, const had::header_t imageHeader, had::image_t& imageMsg, bool isPrintStatus=false)
    {
        Stopwatch sw;
    
        char* rawCompressed = new char[snappy::MaxCompressedLength(bufferSize)];
        size_t rawCompressedLength {0};
        snappy::RawCompress(reinterpret_cast<const char*>(buffer), bufferSize, rawCompressed, &rawCompressedLength);
    
        if(isPrintStatus)
        {
            printf ("strLength=%.3fMB, ", rawCompressedLength / (1024.0 * 1024.0));
            printf ("t_comp=%06.3f, ", sw.toc() * 1000);
        }
    
        // TIPS  without reinterpret_cast, running
        // std::vector< uint8_t >(buffer, buffer+bufferSize); will be very slow
        packageUint8AsMessage(reinterpret_cast<uint8_t*>(rawCompressed), rawCompressedLength,
                            height, width,
                             had::image_compression_t::SNAPPY, had::image_type_t::YUV422,
                             imageHeader, imageMsg);
    
        delete[] rawCompressed;
    }
    
    static bool packageMatGrayAsPng2BitMessage(const cv::Mat &mat, const had::header_t imageHeader, had::image_t &imageMsg, bool isPrintStatus=false)
    {
        png::image<png::gray_pixel_2> png2Bit;
        convertMatGray2Png2Bit(mat, png2Bit);
    
        packagePng2BitAsMessage(png2Bit, imageHeader, imageMsg, isPrintStatus);
    
        return true;
    }
    
    static bool convertMatGray2Png2Bit(const cv::Mat &mat, png::image<png::gray_pixel_2> &png2Bit)
    {
        png2Bit.resize(mat.cols, mat.rows);
    
        for(int r=0; r<mat.rows; ++r)
        {
            for(int c=0; c<mat.cols; ++c)
            {
                auto pixelValue_ori = mat.at<uint8_t>(cv::Point(c, r));
                uint8_t pixelValue = (pixelValue_ori)/64;
                png2Bit.set_pixel(c, r, pixelValue);
            }
        }
    
        return true;
    }
    
    static bool packagePng2BitAsMessage(png::image<png::gray_pixel_2>& png2Bit, const had::header_t imageHeader, had::image_t &imageMsg, bool isPrintStatus=false)
    {
        Stopwatch sw;
    
        std::stringstream dataStream;
        png2Bit.write_stream(dataStream);
    
        const std::string& str = dataStream.str();
        std::vector<uint8_t> buffer(str.begin(), str.end());
    
        packageUint8AsMessage(buffer.data(), buffer.size(),
                              static_cast<int32_t>(png2Bit.get_height()), static_cast<int32_t>(png2Bit.get_width()),
                              had::image_compression_t::NONE, had::image_type_t::PNG,
                              imageHeader, imageMsg, isPrintStatus);
    
        if(isPrintStatus)
        {
            printf ("pngSize=%.3fMB, ", imageMsg.size / (1024.0 * 1024.0));
            printf ("t_lcm=%06.3f, ", sw.toc() * 1000);
        }
    
        return true;
    }
         

    // ---------- decoding -----------
    explicit ImageHelper(const had::image_t& imageType_in) :
        bgr8Buffer_{}
    {
        imageMsg_ = imageType_in;
        temp_ = imageMsg_.data.data();
    }
    
    ~ImageHelper()
    {
        if(bgr8Buffer_)
        { tjFree(bgr8Buffer_); }
    }
    
    bool convertJpegTo8UC3(bool isRGB)
    {
        Stopwatch sw("unJpeg");
    
        int width, height, jpegSubsamp;
        tjhandle tjDecompressHandler = tjInitDecompress();
        tjDecompressHeader2(tjDecompressHandler, imageMsg_.data.data(), static_cast<ulong>(imageMsg_.size), &width, &height, &jpegSubsamp);
    
        bgr8Buffer_ = new uint8_t[width * height * 3];
    
        const auto colorArrangement = isRGB? TJPF_RGB : TJPF_BGR;
        tjDecompress2(tjDecompressHandler, imageMsg_.data.data(), static_cast<ulong>(imageMsg_.size), bgr8Buffer_, width, 0, height, colorArrangement, TJFLAG_FASTDCT);
    
        tjDestroy(tjDecompressHandler);
    
        if(isPrintStatus_)
            printf ("t_unJpeg=%06.3f, ", sw.toc() * 1000);
    
        return true;
    }
    
    void package8UC3ToMat(cv::Mat& mat)
    {
        Stopwatch sw("convert");
    
        cv::Mat(imageMsg_.height, imageMsg_.width, CV_8UC3, bgr8Buffer_).copyTo(mat);
    
        if(isPrintStatus_)
            printf ("t_convert=%06.3f, ", sw.toc() * 1000);
    }
    
    void packageMono8ToMat(cv::Mat& mat)
    {
        Stopwatch sw("convert");
    
        cv::Mat(imageMsg_.height, imageMsg_.width, CV_8UC1, temp_).copyTo(mat);
    
        if(isPrintStatus_)
            printf ("t_convert=%06.3f, ", sw.toc() * 1000);
    }
    
    bool uncompressSnappy()
    {
        Stopwatch sw("uncompress");
    
        if(!(snappy::Uncompress(reinterpret_cast<const char*>(imageMsg_.data.data()), imageMsg_.data.size(), &unCompressed_)))
        { return false; }
    
        temp_ = const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(unCompressed_.c_str()));
    
        if(isPrintStatus_)
            printf ("t_unsnappy=%06.3f, ", sw.toc() * 1000);
    
        return true;
    }
    
    bool packageYUV422AsOpenCVMat(cv::Mat& mat, bool isRGB)
    {
        Stopwatch sw("convert");
    
        mat = cv::Mat(imageMsg_.height, imageMsg_.width, CV_8UC2, temp_);
    
    
        const auto colorArrangement = isRGB? cv::COLOR_YUV2RGB_Y422 : cv::COLOR_YUV2BGR_Y422;
        cv::cvtColor(mat, mat, colorArrangement);
    
        if(isPrintStatus_)
            printf ("t_convert=%06.3f, ", sw.toc() * 1000);
    
        return true;
    }
    
    static bool decode(const had::image_t& imageMsg, cv::Mat& mat, bool isRGB, bool isPrintStatus_in)
    {
        ImageHelper decoder(imageMsg);
        decoder.isPrintStatus_ = isPrintStatus_in;
        auto compression = imageMsg.compression.enumerate;
    
        if(isPrintStatus_in)
            printf ("size=%.3fMB, ", imageMsg.size / (1024.0 * 1024.0));
    
        if(compression != had::image_compression_t::NONE)
        {
            switch(compression)
            {
            case had::image_compression_t::SNAPPY:
                if(!decoder.uncompressSnappy())
                { return false; }
                break;
    
            default:
                std::cout << "the compression method is not supported currently" << std::endl;
                break;
            }
        }
    
        switch(imageMsg.type.enumerate)
        {
        case had::image_type_t::YUV422:
            if(!decoder.packageYUV422AsOpenCVMat(mat, isRGB))
            { return false; }
            break;
    
        case had::image_type_t::JPEG:
            if(!decoder.convertJpegTo8UC3(isRGB))
            { return false; }
            decoder.package8UC3ToMat(mat);
            break;
    
        case had::image_type_t::MONO8:
            decoder.packageMono8ToMat(mat);
            break;
    
        default:
            std::cout << "unsupported format" << std::endl;
            break;
        }
    
        return true;
    }
    
    static bool decodePNG(const had::image_t &imageMsg, std::stringstream& ss)
    {
        std::copy(imageMsg.data.begin(), imageMsg.data.end(), std::ostream_iterator<uint8_t>(ss));
    
        return true;
    }



} ;



















#endif // _IMAGE_HELPER_

