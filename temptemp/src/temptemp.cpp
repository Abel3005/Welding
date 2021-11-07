#include <ros/ros.h>
#include <vector>
#include <numeric>
#include <algorithm>

#include <std_msgs/Float32.h>

#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/ObjectCount.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

nav_msgs::Path sort_path;
geometry_msgs::PoseStamped sort_pose;

ros::Publisher sort_yolo_3d_coords_pub;

void myCallback(std_msgs::Float32 msg){
    //cout<<" data ="<<msg.data<<endl; 
}



double Mean(std::vector<int> array)
{
    double sum = 0.0;

    for (int i = 0; i < array.size(); i++)
        sum += array[i];

    return sum / array.size();

    
}

double Mean_d(std::vector<double> array)
{
    double sum = 0.0;

    for (int i = 0; i < array.size(); i++)
        sum += array[i];

    return sum / array.size();

    
}

double Median_d(std::vector<double> array)
{
    vector<double> sort_array;

    sort_array = array;

    sort(sort_array.begin(), sort_array.end());

    double mid;
    int mid_number;

    mid_number = sort_array.size() / 2;
    mid = sort_array[mid_number];

    return mid;
}


double StandardDeviation(std::vector<int> array)
{
    double meanValue = Mean(array);

    // 배열 요소가 1개밖에 없을 때는
    // NaN(숫자가 아님)이라는 의미로
    // sqrt(-1.0) 을 반환
    int size = array.size();
    if (size < 2)
    {
        return sqrt(-1.0);
    }

    double sum = 0.0;

    for (int i = 0; i < size; i++) {
        double diff = array[i] - meanValue;
        sum += diff * diff;
    }

    return sqrt(sum / (size - 1));
}

double StandardDeviation_d(std::vector<double> array)
{
    double meanValue = Mean_d(array);

    // 배열 요소가 1개밖에 없을 때는
    // NaN(숫자가 아님)이라는 의미로
    // sqrt(-1.0) 을 반환
    int size = array.size();
    if (size < 2)
    {
        return sqrt(-1.0);
    }

    double sum = 0.0;

    for (int i = 0; i < size; i++) {
        double diff = array[i] - meanValue;
        sum += diff * diff;
    }

    return sqrt(sum / (size - 1));
}



// Yolo 2D bbox Data Output
vector<int> vector_x;
vector<int> vector_y;
int frame_count = 0;

void myCallback2(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){

    // cout << "Init callback func 1 " << endl;

    frame_count += 1;
    int detection_count = 0;
    int center_x, center_y;

    //cout<<" bboxes_data = " << msg->bounding_boxes[detection_count] <<endl; 
    //cout << "bboxes_Count = " << msg->bounding_boxes.size() << endl;

    for(int i = 0;i< msg->bounding_boxes.size();i++)
    {
        center_x = (msg->bounding_boxes[i].xmin + msg->bounding_boxes[i].xmax)/2;
        center_y = (msg->bounding_boxes[i].ymin + msg->bounding_boxes[i].ymax)/2;
        //cout << "Center Point = (" << center_x << " , " << center_y << ")" << endl;
    }

    vector_x.push_back(center_x);
    vector_y.push_back(center_y);

    //cout<< vector_x.size() << endl;
    //cout<< vector_y.size() << endl;

    if(vector_y.size() >= 30 && vector_x.size() >= 30)
    {
        frame_count -= 1;
        // cout << "Init callback func 2 " << endl;
        // 변수 초기화

        // Vector -> min max value
        int max_cx = *max_element(vector_x.begin(),vector_x.end());
        int max_cy = *max_element(vector_y.begin(),vector_y.end());
        int min_cx = *min_element(vector_x.begin(),vector_x.end());
        int min_cy = *min_element(vector_y.begin(),vector_y.end());

        // Vector -> Average value
        //double avg_cx = accumulate(vector_x.begin(),vector_x.end(),0.0f)/vector_x.size();

        double avg_cx = Mean(vector_x);
        double avg_cy = Mean(vector_y);

        // Vector -> Standard Deviation value
        double std_cx = StandardDeviation(vector_x);
        double std_cy = StandardDeviation(vector_y);

        // cout << "-----------------------------------------" << endl;
        // cout << "avg (x,y) = ("<< avg_cx << "," << avg_cy <<")" << endl;
        // cout << "std (x,y) = ("<<std_cx<<","<<std_cy << ")" << endl;
        // cout << "min (x,y) = ("<<min_cx<<","<<min_cy << ")" << endl;        
        // cout << "max (x,y) = ("<<max_cx<<","<<max_cy << ")" << endl;

        vector_x.erase(vector_x.begin());        
        vector_y.erase(vector_y.begin());
    }



}
void myCallback3(const darknet_ros_msgs::ObjectCount::ConstPtr& msg){
    //cout<<" bbox_Count = " << msg->count <<endl; 
    //cout << "---------------------------" << endl;
}


// ----Yolo 3D Filter Output----
//Original
vector<double> vector_3dx;
vector<double> vector_3dy;
vector<double> vector_3dz;
vector<double> std_vector_3dx;
vector<double> std_vector_3dy;
vector<double> std_vector_3dz;

//Average Filter
vector<double> mean_vector_3dx;
vector<double> mean_vector_3dy;
vector<double> mean_vector_3dz;
vector<double> avg_std_avg_vector_3dx;
vector<double> avg_std_avg_vector_3dy;
vector<double> avg_std_avg_vector_3dz;

//Median Filter
vector<double> median_vector_3dx;
vector<double> median_vector_3dy;
vector<double> median_vector_3dz;
vector<double> median_std_vector_3dx;
vector<double> median_std_vector_3dy;
vector<double> median_std_vector_3dz;

int frame_count_3d = 0;
int std_avg_count = 0;
int avg_count = 0;
int avg_std_count = 0;

void Sort_3d_Coord(const nav_msgs::Path::ConstPtr& msg){


//////////////////////////////////////////////////////////////////////////////
    //Origin(No Filter) Data 수집 작업 - Filter 별 성능 비교
    frame_count_3d += 1;
    vector_3dx.push_back(msg->poses[0].pose.position.x);
    vector_3dy.push_back(msg->poses[0].pose.position.y);
    vector_3dz.push_back(msg->poses[0].pose.position.z);

    //cout << "---------------------------" <<endl;
    //cout<< "3d_vec_size = " << vector_3dx.size() << vector_3dy.size()<< vector_3dz.size() << endl;

    if(vector_3dy.size() >= 30 && vector_3dx.size() >= 30 && vector_3dz.size() >= 30)
    {
        frame_count_3d -= 1;
        avg_count += 1;

        // Vector -> min max value
        double max_3dx = *max_element(vector_3dx.begin(),vector_3dx.end());
        double max_3dy = *max_element(vector_3dy.begin(),vector_3dy.end());
        double max_3dz = *max_element(vector_3dz.begin(),vector_3dz.end());
        double min_3dx = *min_element(vector_3dx.begin(),vector_3dx.end());
        double min_3dy = *min_element(vector_3dy.begin(),vector_3dy.end());
        double min_3dz = *min_element(vector_3dz.begin(),vector_3dz.end());

        // Vector Average Filter -> Average value
        double avg_3dx = Mean_d(vector_3dx);
        double avg_3dy = Mean_d(vector_3dy);
        double avg_3dz = Mean_d(vector_3dz);

        // Vector Median Filter -> Median value
        double median_3dx = Median_d(vector_3dx);
        double median_3dy = Median_d(vector_3dy);
        double median_3dz = Median_d(vector_3dz);

        mean_vector_3dx.push_back(avg_3dx);
        mean_vector_3dy.push_back(avg_3dy);
        mean_vector_3dz.push_back(avg_3dz);


        median_vector_3dx.push_back(median_3dx);
        median_vector_3dy.push_back(median_3dy);
        median_vector_3dz.push_back(median_3dz);


        if (median_vector_3dx.size() >= 30 && median_vector_3dy.size() >= 30 && median_vector_3dz.size() >= 30)
        {
            double med_std_3dx = StandardDeviation_d(median_vector_3dx);
            double med_std_3dy = StandardDeviation_d(median_vector_3dy);
            double med_std_3dz = StandardDeviation_d(median_vector_3dz);

            median_std_vector_3dx.push_back(med_std_3dx);
            median_std_vector_3dy.push_back(med_std_3dy);
            median_std_vector_3dz.push_back(med_std_3dz);

            if (median_std_vector_3dx.size() >= 30 && median_std_vector_3dy.size() >= 30 && median_std_vector_3dz.size() >= 30)
            {

                double med_std_avg_3dx = Mean_d(median_std_vector_3dx);
                double med_std_avg_3dy = Mean_d(median_std_vector_3dy);
                double med_std_avg_3dz = Mean_d(median_std_vector_3dz);

                cout << "-------------Median Filter Std Avg---------------" << endl;
                cout << "x = " << med_std_avg_3dx << endl;
                cout << "y = " << med_std_avg_3dy << endl;
                cout << "z = " << med_std_avg_3dz << endl;
                cout << "-------------Median Filter Std Avg End---------------" << endl;


                median_std_vector_3dx.erase(median_std_vector_3dx.begin());
                median_std_vector_3dy.erase(median_std_vector_3dy.begin());
                median_std_vector_3dz.erase(median_std_vector_3dz.begin());
            }


            median_vector_3dx.erase(median_vector_3dx.begin());
            median_vector_3dy.erase(median_vector_3dy.begin());
            median_vector_3dz.erase(median_vector_3dz.begin());
        }




        int avg_f_n = 30;
        if(mean_vector_3dx.size() >= avg_f_n && mean_vector_3dy.size() >= avg_f_n && mean_vector_3dz.size() >= avg_f_n)
        {
            avg_count -= 1;
            double avg_std_3dx = StandardDeviation_d(mean_vector_3dx);
            double avg_std_3dy = StandardDeviation_d(mean_vector_3dy);
            double avg_std_3dz = StandardDeviation_d(mean_vector_3dz);

            avg_std_count += 1;

            avg_std_avg_vector_3dx.push_back(avg_std_3dx);
            avg_std_avg_vector_3dy.push_back(avg_std_3dy);
            avg_std_avg_vector_3dz.push_back(avg_std_3dz);
            // 평균필터 표준편차 평균
            if(avg_std_avg_vector_3dx.size() >= avg_f_n && avg_std_avg_vector_3dy.size() >= avg_f_n && avg_std_avg_vector_3dz.size() >= avg_f_n)
            {
                avg_std_count -= 1;
                double avg_std_avg_3dx = Mean_d(avg_std_avg_vector_3dx);
                double avg_std_avg_3dy = Mean_d(avg_std_avg_vector_3dy);
                double avg_std_avg_3dz = Mean_d(avg_std_avg_vector_3dz);

                cout << "-------------Avg Filter Std Avg---------------" << endl;
                cout << "x = " << avg_std_avg_3dx << endl;
                cout << "y = " << avg_std_avg_3dy << endl;
                cout << "z = " << avg_std_avg_3dz << endl;
                cout << "-------------Avg Filter Std Avg End---------------" << endl;

                avg_std_avg_vector_3dx.erase(avg_std_avg_vector_3dx.begin());        
                avg_std_avg_vector_3dy.erase(avg_std_avg_vector_3dy.begin());
                avg_std_avg_vector_3dz.erase(avg_std_avg_vector_3dz.begin());

            }


            mean_vector_3dx.erase(mean_vector_3dx.begin());        
            mean_vector_3dy.erase(mean_vector_3dy.begin());
            mean_vector_3dz.erase(mean_vector_3dz.begin());
        }

        // Vector -> Standard Deviation value
        double std_3dx = StandardDeviation_d(vector_3dx);
        double std_3dy = StandardDeviation_d(vector_3dy);
        double std_3dz = StandardDeviation_d(vector_3dz);

        // cout << "avg (x,y,z) = ("<< avg_3dx << "," << avg_3dy <<"," << avg_3dz << ")" << endl;
        // cout << "std (x,y,z) = ("<< std_3dx << "," << std_3dy <<"," << std_3dz << ")" << endl;
        // cout << "min (x,y,z) = ("<< min_3dx << "," << min_3dy <<"," << min_3dz << ")" << endl;      
        // cout << "max (x,y,z) = ("<< max_3dx << "," << max_3dy <<"," << max_3dz << ")" << endl;  

        vector_3dx.erase(vector_3dx.begin());        
        vector_3dy.erase(vector_3dy.begin());
        vector_3dz.erase(vector_3dz.begin());


        // Original Std -> Avg (30)
        int avg_std_n = 30;
        std_avg_count += 1;

        std_vector_3dx.push_back(std_3dx);
        std_vector_3dy.push_back(std_3dy);
        std_vector_3dz.push_back(std_3dz);

        if(std_vector_3dx.size() >= avg_std_n && std_vector_3dy.size() >= avg_std_n && std_vector_3dz.size() >= avg_std_n)
        {
            std_avg_count -= 1;
            double std_avg_3dx = Mean_d(std_vector_3dx);
            double std_avg_3dy = Mean_d(std_vector_3dy);
            double std_avg_3dz = Mean_d(std_vector_3dz);

            cout << "-------------Origin Std Avg---------------" << endl;
            cout << "x = " << std_avg_3dx << endl;
            cout << "y = " << std_avg_3dy << endl;
            cout << "z = " << std_avg_3dz << endl;
            cout << "-------------Origin Std Avg End---------------" << endl;

            std_vector_3dx.erase(std_vector_3dx.begin());        
            std_vector_3dy.erase(std_vector_3dy.begin());
            std_vector_3dz.erase(std_vector_3dz.begin());
        }



    }



    
    //cout << "3D Coord = " << msg->poses[0].pose.position.x <<endl;
    //cout << "---------------------------" <<endl;

///////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Indexing 작업 
    //cout << msg->poses.size() <<endl;
    
    vector<float> z_list;

    // Check2 -> Z Value list에 저장 
    for(int i =0;i<msg->poses.size();i++)
    {
        z_list.push_back(msg->poses[i].pose.position.x);
    }
    // Z value 정렬
    sort(z_list.begin(),z_list.end());

    // Z 정렬된 것으로 nav_msgs/Path 에 저장
    for (int k =0; k < z_list.size(); k++)
    {
        //cout << z_list[k] << endl;
        for (int j =0; j < z_list.size(); j++)
        {
            if(z_list[k] == msg->poses[j].pose.position.x && msg->poses[j].pose.position.y >= -0.15 && msg->poses[j].pose.position.y <= 0.15)
            {
                //
                //cout << "Find Same & Detect ROI position" << endl;
            }
            sort_path.poses.push_back(msg->poses[j]);

        }
    }

    sort_yolo_3d_coords_pub.publish(sort_path);
    sort_path.poses.clear();
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
}

int main(int argc, char** argv){

    ros::init(argc, argv, "temptemp");
    ros::NodeHandle nh;

    ros::Rate l(20);
    ros::Subscriber abc = nh.subscribe("/my_msg",10 ,myCallback);
    // Yolo 2D 
    ros::Subscriber yolo_bbox_boxes_sub = nh.subscribe("/darknet_ros/bounding_boxes",10 ,myCallback2);
    ros::Subscriber yolo_bbox_box_sub = nh.subscribe("/darknet_ros/found_object",10 ,myCallback3);

    // Yolo 3D 
    ros::Subscriber yolo_3d_coords_sub = nh.subscribe("/darknet_3d/check2",10,Sort_3d_Coord);
    //ros::Subscriber check_recieve_3d_sub = nh.subscribe("/dar")

    sort_yolo_3d_coords_pub = nh.advertise<nav_msgs::Path>("/darknet_3d/sorted_poses",100);

    while(ros::ok()){
        l.sleep();
        ros::spinOnce();
        //cout<<"In LOOP"<<endl;
    }
    return 0;


}
