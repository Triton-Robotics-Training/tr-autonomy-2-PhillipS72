#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/empty.hpp"
#include "opencv2/opencv.hpp"

class ImageAngleController : public rclcpp::Node
{
public:
    ImageAngleController()
        : Node("image_angle_controller"), is_tracking_(false), score_count_(0), angle_(0.0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float32>("desired_angle", 10);
        scored_point_publisher_ = this->create_publisher<std_msgs::msg::Empty>("score_point", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "robotcam", 10,
            std::bind(&ImageAngleController::image_callback, this, std::placeholders::_1));
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat image = cv_ptr->image;
        cv::Mat hsv_image;
        cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

        cv::Scalar lower_red1(0, 100, 100);
        cv::Scalar upper_red1(10, 255, 255);
        cv::Scalar lower_red2(160, 100, 100);
        cv::Scalar upper_red2(180, 255, 255);

        cv::Mat mask1, mask2, red_mask;
        cv::inRange(hsv_image, lower_red1, upper_red1, mask1);
        cv::inRange(hsv_image, lower_red2, upper_red2, mask2);
        cv::add(mask1, mask2, red_mask);

        cv::Moments moments = cv::moments(red_mask, true);
        int red_pixel_count = static_cast<int>(moments.m00);
        double cx = moments.m10 / (red_pixel_count > 0 ? red_pixel_count : 1);

        if (red_pixel_count > 0)
        {
            double image_center_x = image.cols / 2.0;
            double target_angle = (image_center_x - cx) / image_center_x * (M_PI / 2.0);

           
            const double adjustment_rate = 0.1; 
            if (std::abs(target_angle - angle_) > adjustment_rate)
            {
                angle_ += (target_angle - angle_) > 0 ? adjustment_rate : -adjustment_rate;
            }
            else
            {
                angle_ = target_angle; 
            }

            if (!is_tracking_)
            {
                is_tracking_ = true; 
                RCLCPP_INFO(this->get_logger(), "Locked onto red block. Angle: %f", angle_);
            }

            auto msg_out = std_msgs::msg::Float32();
            msg_out.data = angle_;
            publisher_->publish(msg_out);
        }
        else
        {
            if (is_tracking_)
            {
                RCLCPP_WARN(this->get_logger(), "Lost red pixels. Resetting angle to 0...");
                angle_ = 0.0; 
                auto msg_out = std_msgs::msg::Float32();
                msg_out.data = angle_;
                publisher_->publish(msg_out);
                is_tracking_ = false; 
            }
        }

        
        if (is_tracking_ && red_pixel_count > 0)
        {
            double image_center_x = image.cols / 2.0;
            if (std::abs(cx - image_center_x) < 60)
            {
                if (score_count_ == 0)
                {
                    last_score_time_ = this->now();
                    score_count_++;
                }
                else if ((this->now() - last_score_time_).seconds() > 2.0) 
                {
                    auto empty_msg = std_msgs::msg::Empty();
                    scored_point_publisher_->publish(empty_msg);
                    RCLCPP_INFO(this->get_logger(), "Point scored!");

                    is_tracking_ = false;
                    score_count_ = 0;
                }
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr scored_point_publisher_;

    bool is_tracking_;
    double angle_;
    int score_count_;
    rclcpp::Time last_score_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageAngleController>());
    rclcpp::shutdown();
    return 0;
}
