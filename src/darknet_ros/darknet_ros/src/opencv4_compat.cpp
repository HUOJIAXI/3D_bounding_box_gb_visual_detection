// OpenCV 4 compatibility layer for deprecated C API functions
#include <opencv2/core/version.hpp>

#if CV_MAJOR_VERSION >= 4
#include <opencv2/opencv.hpp>
#include <opencv2/core/types_c.h>
#include <cstring>

extern "C" {

// cvLoadImage replacement
IplImage* cvLoadImage(const char* filename, int flags) {
    int cv_flags;
    if (flags == 0) {
        cv_flags = cv::IMREAD_GRAYSCALE;
    } else if (flags == 1) {
        cv_flags = cv::IMREAD_COLOR;
    } else {
        cv_flags = cv::IMREAD_UNCHANGED;
    }

    cv::Mat* img = new cv::Mat(cv::imread(filename, cv_flags));
    if (img->empty()) {
        delete img;
        return nullptr;
    }

    // Create IplImage header that points to the Mat data
    // We need to keep the Mat alive, so we allocate it on heap
    IplImage* ipl_img = new IplImage;
    *ipl_img = cvIplImage(*img);

    // Allocate new memory and copy data to ensure it persists
    IplImage* result = new IplImage;
    memcpy(result, ipl_img, sizeof(IplImage));
    result->imageData = new char[img->total() * img->elemSize()];
    memcpy(result->imageData, img->data, img->total() * img->elemSize());

    delete ipl_img;
    delete img;

    return result;
}

// cvSaveImage replacement
int cvSaveImage(const char* filename, const CvArr* image, const int* params) {
    IplImage* ipl = (IplImage*)image;
    cv::Mat mat(cv::Size(ipl->width, ipl->height),
                CV_8UC(ipl->nChannels),
                ipl->imageData,
                ipl->widthStep);

    std::vector<int> compression_params;
    if (params != nullptr) {
        // Copy params to vector (assuming null-terminated or known length)
        // For simplicity, we'll use default params
    }

    return cv::imwrite(filename, mat, compression_params) ? 1 : 0;
}

} // extern "C"

#endif // CV_MAJOR_VERSION >= 4
