//this node will analyze the image and request drive services to the drive bot

#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

ros::ServiceClient client; //global client declaration

void drive_robot(float lin_x, float ang_z)
{
	ROS_INFO_STREAM("Driving Robot towards the ball");
	ball_chaser::DriveToTarget srv;
	srv.request.linear_x = lin_x;
	srv.request.angular_z = ang_z;

	if(!client.call(srv))
	{
		ROS_ERROR("Failed to call services drive_robot");
	}
}

void process_image_callback(const sensor_msgs::Image img)
{
	
	int count(0);
	float lin_x(0.0);
	float ang_z(0.0);
	float offset(0.0);

	for (int i = 0; i < img.height - 1; ++i)
	{
		for (int j = 0; j < img.step - 1; ++j)
		{
			if (img.data[i*img.step + j] == 255 && img.data[i*img.step + j + 1] == 255 && img.data[i*img.step + j + 2] == 255)
			{
				count++;
				offset += j - (img.step)/2.0;
			}
		}
	}


	if (count == 0)
	{
		lin_x = 0.0;
                ang_z = 0.5;
	}

	else
	{
		lin_x = 1.0;
		ang_z = -4.0*offset/(count*(img.step)/2.0);
	}

	drive_robot(lin_x,ang_z);
}




int main(int argc, char **argv)
{
	ros::init(argc, argv, "process_image");
	ros::NodeHandle n;
    
    //define a client service capable of requesting services from command_robot
	client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

	//subsbribe to /camera/rgb/image_raw topic to read data inside process_image_callback
	ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

	ros::spin(); //handle ROS communication events
 
	return 0;
}
