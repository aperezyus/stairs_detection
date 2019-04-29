/**
* This file is part of stairs_detection.
*
* Copyright (C) 2019 Alejandro Pérez Yus <alperez at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/aperezyus/stairs_detection>
*
* stairs_detection is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* stairs_detection is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with stairs_detection. If not, see <http://www.gnu.org/licenses/>.
*/

#include "RGBD/plane.h"
//#include "custom_functions.h"

bool sortPlanesBySize(const Plane &lhs, const Plane &rhs)
{
    return lhs.cloud->points.size() > rhs.cloud->points.size(); //> los ordena de mayor a menor, < al revés
}

void Plane::getCoeffs2Floor(Eigen::Affine3d c2f)
{
  coeffs2f = c2f.inverse().matrix().transpose().cast<float>()*coeffs;

}

void Plane::getCentroid()
{
	Eigen::Vector4f vector_centroid;
	pcl::compute3DCentroid(*cloud,vector_centroid);
	centroid = pcl::PointXYZ (vector_centroid[0], vector_centroid[1], vector_centroid[2]);
}
	
void Plane::getCentroid2Floor(Eigen::Affine3d c2f)
{
	centroid2f.x = static_cast<float> (c2f.matrix() (0, 0) * centroid.x + c2f.matrix() (0, 1) * centroid.y + c2f.matrix() (0, 2) * centroid.z + c2f.matrix() (0, 3));
	centroid2f.y = static_cast<float> (c2f.matrix() (1, 0) * centroid.x + c2f.matrix() (1, 1) * centroid.y + c2f.matrix() (1, 2) * centroid.z + c2f.matrix() (1, 3));
	centroid2f.z = static_cast<float> (c2f.matrix() (2, 0) * centroid.x + c2f.matrix() (2, 1) * centroid.y + c2f.matrix() (2, 2) * centroid.z + c2f.matrix() (2, 3));

}

void Plane::getCloud2Floor(Eigen::Affine3d c2f)
{
  cloud2f.reset(new pcl::PointCloud<pcl::PointXYZ>());

  pcl::transformPointCloud(*cloud,*cloud2f,c2f);
}

void Plane::getContour()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud);
    pcl::ModelCoefficients::Ptr model_coeffs (new pcl::ModelCoefficients);
    model_coeffs->values.resize(4);

    model_coeffs->values[0] = coeffs(0);
    model_coeffs->values[1] = coeffs(1);
    model_coeffs->values[2] = coeffs(2);
    model_coeffs->values[3] = coeffs(3);
    proj.setModelCoefficients (model_coeffs);
    proj.filter (*cloud_projected);

    // Se obtiene la concave hull
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setAlpha (0.1);
    chull.setInputCloud (cloud_projected);
    contour.reset(new pcl::PointCloud<pcl::PointXYZ>);
    chull.reconstruct (*contour);

    // vPlanes[Q].contour = cloud_hull_floor_aux;
}

void Plane::getContour2Floor(Eigen::Affine3d c2f)
{
  contour2f.reset(new pcl::PointCloud<pcl::PointXYZ>());

  pcl::transformPointCloud(*contour,*contour2f,c2f);
}

void Plane::getPrincipalDirections()
{
	Eigen::Matrix3f covariance;
	Eigen::Vector4f eigen_centroid(centroid.x,centroid.y,centroid.z,1);
	pcl::computeCovarianceMatrixNormalized(*cloud,eigen_centroid,covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance,Eigen::ComputeEigenvectors);
	eigDx = eigen_solver.eigenvectors();				    
	eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));
}


void Plane::getMeasurements()
{
  if (eigDx.isZero(0))
    this->getPrincipalDirections();

  // move the points to the that reference frame
    Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
    p2w.block<3,3>(0,0) = eigDx.transpose();
    p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.getVector3fMap().head<3>());
    pcl::PointCloud<pcl::PointXYZ> cPoints;
    pcl::transformPointCloud(*cloud, cPoints, p2w);

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(cPoints, min_pt, max_pt);

    width = max_pt.z - min_pt.z;
    length = max_pt.y - min_pt.y;
    height = max_pt.x - min_pt.x;

    Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());
    center.x = mean_diag(0) + centroid.x;
    center.y = mean_diag(1) + centroid.y;
    center.z = mean_diag(2) + centroid.z;

}

void Plane::getMeasurements(Eigen::Matrix3f c2m)
{

    Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
    //está metiéndole la inversa, ojo, va por filas (bueno, la traspuesta)
    p2w.block<1,3>(0,0) = c2m.col(0);
    p2w.block<1,3>(1,0) = c2m.col(1);
    p2w.block<1,3>(2,0) = c2m.col(2);
    p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.getVector3fMap().head<3>());

    Eigen::Affine3d w2p = Eigen::Translation3d(centroid.getVector3fMap().cast<double>()) * Eigen::AngleAxisd(c2m.cast<double>());

    pcl::PointCloud<pcl::PointXYZ> cPoints;
    pcl::transformPointCloud(*cloud, cPoints, p2w);

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(cPoints, min_pt, max_pt);

    // In these projection matrix w/ manhattan directions, the y go upwards (height) and
    // the width should be in X to match the c2f
    width = max_pt.x - min_pt.x;
    height = max_pt.y - min_pt.y;
    length = max_pt.z - min_pt.z;

    Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());

    pcl::transformPoint(mean_diag,mean_diag,w2p.cast<float>());
    center = pcl::PointXYZ(mean_diag(0),mean_diag(1),mean_diag(2));//getPointXYZ(mean_diag);

//     std::cout << width << std::endl << height << std::endl << length << std::endl << std::endl;

 //    std::cout << min_pt.x << " " << min_pt.y << " " << min_pt.z << std::endl;
  // std::cout << max_pt.x << " " << max_pt.y << " " << max_pt.z << std::endl << std::endl;
}

void Plane::getMeasurements(Eigen::Affine3d & c2m)
{
  // Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
 //    p2w.block<1,3>(0,0) = c2m.rotation().col(0).cast<float>();
 //    p2w.block<1,3>(1,0) = c2m.rotation().col(1).cast<float>();
 //    p2w.block<1,3>(2,0) = c2m.rotation().col(2).cast<float>();
 //    p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) *c2m.translation().cast<float>());

 //    Eigen::Affine3d w2p = c2m;

    pcl::PointCloud<pcl::PointXYZ> cPoints;
    // pcl::transformPointCloud(*cloud, cPoints, p2w);
    pcl::transformPointCloud(*cloud, cPoints, c2m);

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(cPoints, min_pt, max_pt);
    // std::cout << "x: " << min_pt.x <<  " y: " << min_pt.y <<  " z: " << min_pt.z <<  std::endl;
    // std::cout << "x: " << max_pt.x <<  " y: " << max_pt.y <<  " z: " << max_pt.z <<  std::endl;

    // In these projection matrix w/ manhattan directions, the y go upwards (height) and
    // the width should be in X to match the c2f
    width = max_pt.x - min_pt.x;
    height = max_pt.y - min_pt.y;
    length = max_pt.z - min_pt.z;

    Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());

    pcl::transformPoint(mean_diag,mean_diag,c2m.inverse().cast<float>());
    center = pcl::PointXYZ(mean_diag(0),mean_diag(1),mean_diag(2));//getPointXYZ(mean_diag);
}

void Plane::getMeasurements(Eigen::Matrix3f c2m, pcl::PointCloud<pcl::PointXYZ>::Ptr custom_cloud)
{
    Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
    p2w.block<1,3>(0,0) = c2m.col(0);
    p2w.block<1,3>(1,0) = c2m.col(1);
    p2w.block<1,3>(2,0) = c2m.col(2);
    p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.getVector3fMap().head<3>());

  Eigen::Affine3d w2p = Eigen::Translation3d(centroid.getVector3fMap().cast<double>()) * Eigen::AngleAxisd(c2m.cast<double>());

    pcl::PointCloud<pcl::PointXYZ> cPoints;
    pcl::transformPointCloud(*custom_cloud, cPoints, p2w);

    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(cPoints, min_pt, max_pt);


    // In these projection matrix w/ manhattan directions, the y go upwards (height) and
    // the width should be in X to match the c2f
    width = max_pt.x - min_pt.x;
    height = max_pt.y - min_pt.y;
    length = max_pt.z - min_pt.z;

  Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());

    pcl::transformPoint(mean_diag,mean_diag,w2p.cast<float>());
    center = pcl::PointXYZ(mean_diag(0),mean_diag(1),mean_diag(2));//getPointXYZ(mean_diag);
}

void Plane::getVertices()
{
  // const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());
  // Eigen::Vector3f mean_diag_absolute = eigDx*mean_diag+ centroid.getVector3fMap().head<3>();

  vertices.reset(new pcl::PointCloud<pcl::PointXYZ>);

  Eigen::Vector3f pt1, pt2, pt3, pt4;

  pt1 = center.getVector3fMap()  + (width/2)*eigDx.col(2) + (length/2)*eigDx.col(1);
  pt2 = center.getVector3fMap()  - (width/2)*eigDx.col(2) + (length/2)*eigDx.col(1);
  pt3 = center.getVector3fMap()  - (width/2)*eigDx.col(2) - (length/2)*eigDx.col(1);
  pt4 = center.getVector3fMap()  + (width/2)*eigDx.col(2) - (length/2)*eigDx.col(1);

  vertices->push_back(pcl::PointXYZ(pt1(0),pt1(1),pt1(2)));
  vertices->push_back(pcl::PointXYZ(pt2(0),pt2(1),pt2(2)));
  vertices->push_back(pcl::PointXYZ(pt3(0),pt3(1),pt3(2)));
  vertices->push_back(pcl::PointXYZ(pt4(0),pt4(1),pt4(2)));
  vertices->push_back(pcl::PointXYZ(center.getVector3fMap() (0),center.getVector3fMap() (1),center.getVector3fMap() (2)));
}

void Plane::getVertices(Eigen::Matrix3f c2m)
{
    // std::cout << "x: " << min_pt.x <<  " y: " << min_pt.y <<  " z: " << min_pt.z <<  std::endl;
    // std::cout << "x: " << max_pt.x <<  " y: " << max_pt.y <<  " z: " << max_pt.z <<  std::endl;

  // const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());
  // Eigen::Vector3f mean_diag_absolute = c2m*mean_diag;//+ centroid.getVector3fMap().head<3>();

  // std::cout << mean_diag << std::endl << std::endl;
  // std::cout << mean_diag_absolute << std::endl << std::endl;

  vertices.reset(new pcl::PointCloud<pcl::PointXYZ>);

  Eigen::Vector3f pt1, pt2, pt3, pt4;

  // pt1 = mean_diag + (width/2)*c2m.col(0) + (length/2)*c2m.col(2);
  // pt2 = mean_diag - (width/2)*c2m.col(0) + (length/2)*c2m.col(2);
  // pt3 = mean_diag - (width/2)*c2m.col(0) - (length/2)*c2m.col(2);
  // pt4 = mean_diag + (width/2)*c2m.col(0) - (length/2)*c2m.col(2);

    pt1 = center.getVector3fMap() + (width/2)*c2m.col(0) + (length/2)*c2m.col(2);
    pt2 = center.getVector3fMap() - (width/2)*c2m.col(0) + (length/2)*c2m.col(2);
    pt3 = center.getVector3fMap() - (width/2)*c2m.col(0) - (length/2)*c2m.col(2);
    pt4 = center.getVector3fMap() + (width/2)*c2m.col(0) - (length/2)*c2m.col(2);
 //    pt1 = center.getVector3fMap() + Eigen::Vector3f(width/2, 0, length/2);
 //    pt1 = center.getVector3fMap() + Eigen::Vector3f(-width/2, 0, length/2);
 //    pt1 = center.getVector3fMap() + Eigen::Vector3f(-width/2, 0, -length/2);
  // pt1 = center.getVector3fMap() + Eigen::Vector3f(width/2, 0, -length/2);

  vertices->push_back(pcl::PointXYZ(pt1(0),pt1(1),pt1(2)));
  vertices->push_back(pcl::PointXYZ(pt2(0),pt2(1),pt2(2)));
  vertices->push_back(pcl::PointXYZ(pt3(0),pt3(1),pt3(2)));
  vertices->push_back(pcl::PointXYZ(pt4(0),pt4(1),pt4(2)));
  vertices->push_back(pcl::PointXYZ(center.getVector3fMap()(0),center.getVector3fMap()(1),center.getVector3fMap()(2)));
}







float Plane::getContourArea()
{
     if (contour == NULL)
     {
        this->getContour();
     }

    float contour_area = pcl::calculatePolygonArea(*contour);

    return contour_area;
}

float Plane::getRectangleArea()
{
    // if ((width == 0) and (length == 0))
        this->getMeasurements();

    float rectangle_area = width*length;

    return rectangle_area;
}

float Plane::getRectangleArea(Eigen::Matrix3f c2m)
{
    // if ((width == 0) and (length == 0))
        this->getMeasurements(c2m);

    float rectangle_area = width*length;

    return rectangle_area;
}

float Plane::getExtent()
{

    // std::cout << "Contour area: " << this->getContourArea() << std::endl;
    // std::cout << "Rectangle area: " << this->getRectangleArea() << std::endl;
    float extent = this->getContourArea()/this->getRectangleArea();

    return extent;
}



float Plane::getExtent(Eigen::Matrix3f c2m)
{

    float extent = this->getContourArea()/this->getRectangleArea(c2m);

    return extent;
}


//void Plane::getStairDirFromPCA (Eigen::Affine3d c2f)
//{
//    if (eigDx.isZero(0))
//        this->getPrincipalDirections();

//    Eigen::Vector3f best_normal2c = eigDx.col(2);
//    Eigen::Vector3f best_normal = c2f.rotation().cast<float>()*best_normal2c;

//    float angle_x = acos(Eigen::Vector2f(best_normal(0),best_normal(2)).normalized().dot(Eigen::Vector2f(1,0)));
//    float angle_z = acos(Eigen::Vector2f(best_normal(0),best_normal(2)).normalized().dot(Eigen::Vector2f(0,1)));
////    float angle_x = getHorizontalAngle(best_normal,Eigen::Vector3f(1,0,0));
////    float angle_z = getHorizontalAngle(best_normal,Eigen::Vector3f(0,0,1));

//    float threshold = 45*M_PI/180;

//    Eigen::Matrix3f eigDx2f = Eigen::Matrix3f::Identity();

//    if ((fabs(2*M_PI-angle_z) < threshold) or (angle_z < threshold))
//    {
//        eigDx2f.col(0) = Eigen::Vector3f(best_normal(2),0, -best_normal(0));
//        eigDx2f.col(2) = Eigen::Vector3f(best_normal(0),0,best_normal(2));
//        eigDx2f.col(1) = eigDx2f.col(2).cross(eigDx2f.col(0));
//    }
//    else if (fabs(M_PI-angle_z) < threshold)
//    {
//        eigDx2f.col(0) = Eigen::Vector3f(-best_normal(2),0,best_normal(0));
//        eigDx2f.col(2) = Eigen::Vector3f(-best_normal(0),0,-best_normal(2));
//        eigDx2f.col(1) = eigDx2f.col(2).cross(eigDx2f.col(0));
//    }
//    else
//    {
//        if ((fabs(2*M_PI-angle_x) < threshold) or (angle_x < threshold))
//        {
//            eigDx2f.col(0) = Eigen::Vector3f(best_normal(0),0, best_normal(2));
//            eigDx2f.col(2) = Eigen::Vector3f(-best_normal(2),0, best_normal(0));
//            eigDx2f.col(1) = eigDx2f.col(2).cross(eigDx2f.col(0));

//        }
//        else if (fabs(M_PI-angle_x) < threshold)
//        {
//            eigDx2f.col(0) = Eigen::Vector3f(-best_normal(0),0,-best_normal(2));
//            eigDx2f.col(2) = Eigen::Vector3f(best_normal(2),0,-best_normal(0));
//            eigDx2f.col(1) = eigDx2f.col(2).cross(eigDx2f.col(0));

//        }
//        else
//        {
//            eigDx2f.col(0) = Eigen::Vector3f(best_normal(0),0,best_normal(2));
//            eigDx2f.col(2) = Eigen::Vector3f(-best_normal(2),0,best_normal(0));
//            eigDx2f.col(1) = eigDx2f.col(2).cross(eigDx2f.col(0));
//        }
//    }

//    Eigen::Affine3f f2c_aux = c2f.inverse().cast<float>();
//    f2c_aux.translation() = Eigen::Vector3f(0,0,0);

//    Eigen::Matrix3f step_dir;

//    step_dir.col(0) = (f2c_aux)*eigDx2f.col(0);
//    step_dir.col(1) = (f2c_aux)*eigDx2f.col(1);
//    step_dir.col(2) = (f2c_aux)*eigDx2f.col(2);

//    eigDx = step_dir;
//}

std::vector<cv::Point> Plane::projectContourPointsImg (cv::Mat img, cv::Mat K, Eigen::Affine3d T, bool absolute)
{
    std::vector<cv::Point> points2D;
    //    points2D = cv::Mat(cloud->points.size(),2,CV_32FC1,cv::Scalar(0.0));

    if (absolute)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr contour_aux (new pcl::PointCloud<pcl::PointXYZ>());
      pcl::transformPointCloud(*contour,*contour_aux,T);

      for (int i = 0; i<contour_aux->points.size(); i++)
      {
        cv::Point point2D(int(round(K.at<double>(0,0)*contour_aux->points[i].x/contour_aux->points[i].z + K.at<double>(0,2))),int(round(K.at<double>(1,1)*contour_aux->points[i].y/contour_aux->points[i].z + K.at<double>(1,2))));
        if (point2D.x >= 0 && point2D.x < img.cols && point2D.y >= 0 && point2D.y < img.rows)
          points2D.push_back(point2D);
      }
    }
    else
    {
      for (int i = 0; i<contour->points.size(); i++)
      {
        cv::Point point2D(int(round(K.at<double>(0,0)*contour->points[i].x/contour->points[i].z + K.at<double>(0,2))),int(round(K.at<double>(1,1)*contour->points[i].y/contour->points[i].z + K.at<double>(1,2))));
        if (point2D.x >= 0 && point2D.x < img.cols && point2D.y >= 0 && point2D.y < img.rows)
          points2D.push_back(point2D);
      }
    }

//    std::cout << points2D << std::endl;

    return points2D;
}

void Plane::plotPolyInImg(cv::Mat img, cv::Mat K, Eigen::Affine3d T, bool absolute, cv::Scalar color)
{

  // Get poly
  std::vector<cv::Point> poly = projectContourPointsImg(img,K,T,absolute);

  // Plot poly
    int n_vertices[] = {poly.size()};
    cv::Point rook_points[1][poly.size()];
    for (int i = 0; i < poly.size(); i++)
      rook_points[0][i] = poly[i];

    const cv::Point* ppt[1] = { rook_points[0] };

    cv::fillPoly( img, ppt, n_vertices, 1, color, 8 );
}

void Plane::fixHolesConcaveHull()
{
    double voxel_size = 0.04;
    double min_length = 5*voxel_size;

    if (contour->points.size() < 5)
        return;

    std::vector<int> ids; // indexes whose distance to the following one is greater than min_length

    for (int i = 0; i<contour->points.size(); i++)
    {
        float distance;
        if (i < contour->points.size()-1)
        {
            distance =    sqrt(     (contour->points[i].x - contour->points[i+1].x)*(contour->points[i].x - contour->points[i+1].x) +
                                    (contour->points[i].y - contour->points[i+1].y)*(contour->points[i].y - contour->points[i+1].y) +
                                    (contour->points[i].z - contour->points[i+1].z)*(contour->points[i].z - contour->points[i+1].z)
                                );

//            std::cout << distance << std::endl;
        }
        else
        {
            distance =    sqrt(     (contour->points[i].x - contour->points[0].x)*(contour->points[i].x - contour->points[0].x) +
                                    (contour->points[i].y - contour->points[0].y)*(contour->points[i].y - contour->points[0].y) +
                                    (contour->points[i].z - contour->points[0].z)*(contour->points[i].z - contour->points[0].z)
                                );

        }
//        std::cout << " Distance: " << distance << " i: " << i << std::endl;
        if (distance > min_length)
            ids.push_back(i);
    }

    if (ids.size() == 2)
    {
//        std::cout << "change! " << ids[0] << " " << ids[1] << std::endl;
//        std::cout << contour->points.size() << std::endl;
        contour->insert(contour->points.begin()+ids[1]+1,contour->points[ids[0]+1]);
        contour->insert(contour->points.begin()+ids[1]+2,contour->points[ids[0]]);
//        std::cout << contour->points.size() << std::endl;
    }
}

