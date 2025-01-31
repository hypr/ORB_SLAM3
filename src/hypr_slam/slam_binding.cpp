#include <pybind11/pybind11.h>
#include <pybind11/stl.h>   // Automatic STL type conversions
#include <pybind11/numpy.h> // Numpy array support
#include <System.h>
#include <opencv2/core/core.hpp>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <unistd.h>

namespace py = pybind11;

class PySLAM
{
public:
    PySLAM(const std::string &vocab_path, const std::string &settings_path, bool enable_viewer = false)
    {
        slam_system = new ORB_SLAM3::System(vocab_path, settings_path, ORB_SLAM3::System::MONOCULAR, enable_viewer);
        image_scale = slam_system->GetImageScale();
    }

    ~PySLAM()
    {
        if (slam_system)
        {
            slam_system->Shutdown();
            delete slam_system;
        }
    }

    py::dict process_image(py::array_t<unsigned char> image, double timestamp)
    {
        // Convert numpy array to cv::Mat
        py::buffer_info buf = image.request();
        cv::Mat im(buf.shape[0], buf.shape[1], CV_8UC3, (unsigned char *)buf.ptr);

        // Resize if needed
        if (image_scale != 1.f)
        {
            int width = im.cols * image_scale;
            int height = im.rows * image_scale;
            cv::resize(im, im, cv::Size(width, height));
        }

        // Track and get pose
        Sophus::SE3f Tcw = slam_system->TrackMonocular(im, timestamp);

        py::dict result;
        // Check if tracking was successful (matrix is valid)
        if (Tcw.matrix().determinant() != 0)
        {
            // Convert SE3 pose to numpy array
            Eigen::Matrix4f pose_mat = Tcw.matrix();
            std::vector<float> pose_data(pose_mat.data(), pose_mat.data() + 16);
            result["pose"] = py::array_t<float>({4, 4}, pose_data.data());
            result["status"] = "OK";
        }
        else
        {
            result["status"] = "Lost";
        }

        return result;
    }

    py::dict get_trajectory(bool keyframes_only = false)
    {
        // Create a temporary file using mkstemp
        char filename_template[] = "/tmp/slam_trajectory_XXXXXX";
        int fd = mkstemp(filename_template);
        if (fd == -1)
        {
            throw std::runtime_error("Failed to create temporary file");
        }
        close(fd); // Close file descriptor, we'll use the filename

        // Save trajectory to temp file
        if (keyframes_only)
        {
            slam_system->SaveKeyFrameTrajectoryTUM(filename_template);
        }
        else
        {
            slam_system->SaveTrajectoryTUM(filename_template);
        }

        // Read the file back
        std::vector<double> timestamps;
        std::vector<float> poses; // Will store tx, ty, tz, qx, qy, qz, qw for each pose

        std::ifstream file(filename_template);
        std::string line;
        while (std::getline(file, line))
        {
            double timestamp, tx, ty, tz, qx, qy, qz, qw;
            std::istringstream iss(line);
            if (!(iss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw))
                continue;

            timestamps.push_back(timestamp);
            poses.push_back(tx);
            poses.push_back(ty);
            poses.push_back(tz);
            poses.push_back(qx);
            poses.push_back(qy);
            poses.push_back(qz);
            poses.push_back(qw);
        }
        file.close();

        // Delete temporary file
        std::remove(filename_template);

        // Return as dictionary
        py::dict result;
        result["timestamps"] = py::array_t<double>(timestamps.size(), timestamps.data());
        result["poses"] = py::array_t<float>(std::vector<size_t>{timestamps.size(), 7}, poses.data()); // Each pose is [tx, ty, tz, qx, qy, qz, qw]

        return result;
    }

private:
    ORB_SLAM3::System *slam_system;
    float image_scale;
};

PYBIND11_MODULE(hypr_slam, m)
{
    m.doc() = "Python binding for ORB-SLAM3"; // module docstring

    py::class_<PySLAM>(m, "SLAM")
        .def(py::init<const std::string &, const std::string &, bool>(),
             py::arg("vocab_path"),
             py::arg("settings_path"),
             py::arg("enable_viewer") = false)
        .def("process_image", &PySLAM::process_image)
        .def("get_trajectory", &PySLAM::get_trajectory,
             py::arg("keyframes_only") = false);
}