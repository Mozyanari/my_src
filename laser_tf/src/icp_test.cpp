#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

using namespace std;

void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

int main (int argc, char** argv){

    //点群を宣言する。
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

    //点群の移動量を設定
    double x_offset=0.9f;
    double y_offset=0.5f;

    //点群cloud_inにデータを入力
    cloud_in->width    = 6;
    cloud_in->height   = 1;
    cloud_in->is_dense = false;
    cloud_in->points.resize (cloud_in->width * cloud_in->height);

    for (size_t i = 0; i < cloud_in->points.size (); ++i)   {
        cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }

    std::cout << "Saved " << cloud_in->points.size () << " data points to input:"<< std::endl;
    for (size_t i = 0; i < cloud_in->points.size (); ++i) std::cout << "    " <<
    cloud_in->points[i].x << " " <<
    cloud_in->points[i].y << " " <<
    cloud_in->points[i].z << std::endl;
    *cloud_out = *cloud_in;
    std::cout << "size:" << cloud_out->points.size() << std::endl;

    //点群を移動させcloud_outを作る。
    for (size_t i = 0; i < cloud_in->points.size (); ++i){
        cloud_out->points[i].x = cloud_in->points[i].x + x_offset;
        cloud_out->points[i].y = cloud_in->points[i].y + y_offset;

    }
    std::cout << "Transformed " << cloud_in->points.size () << " data points:"  << std::endl;

    for (size_t i = 0; i < cloud_out->points.size (); ++i)
    std::cout << "    " <<
    cloud_out->points[i].x << " " <<
    cloud_out->points[i].y << " " <<
    cloud_out->points[i].z << std::endl;

    //cloud_inとcloud_outをICPアルゴズムにより変換matrixを求める。
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputCloud(cloud_in);
    icp.setInputTarget(cloud_out);

    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    //変換matrixを表示する
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
    transformation_matrix = icp.getFinalTransformation ().cast<double>();
    print4x4Matrix (transformation_matrix);

    return (0);
}
