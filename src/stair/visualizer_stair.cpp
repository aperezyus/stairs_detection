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

#include "stair/visualizer_stair.h"


void ViewerStair::drawStairAxis (Stair stair, std::string stair_type) {
    std::stringstream sx,sy,sz;
    sx << stair_type << "x";
    sy << stair_type << "y";
    sz << stair_type << "z";
    const std::string tmpx = sx.str();
    const char* name = tmpx.c_str();
    cloud_viewer_.removeShape(name);
    cloud_viewer_.addArrow (pcl::PointXYZ(stair.initial_point.x+stair.stair_dir.col(0)(0)*1, stair.initial_point.y+stair.stair_dir.col(0)(1)*1, stair.initial_point.z+stair.stair_dir.col(0)(2)*1), stair.initial_point, 1.0f, 0.0f, 0.0f, false, name);
    const std::string tmpy = sy.str();
    name = tmpy.c_str();
    cloud_viewer_.removeShape(name);
    cloud_viewer_.addArrow (pcl::PointXYZ(stair.initial_point.x+stair.stair_dir.col(1)(0)*1, stair.initial_point.y+stair.stair_dir.col(1)(1)*1, stair.initial_point.z+stair.stair_dir.col(1)(2)*1), stair.initial_point, 0.0f, 1.0f, 0.0f, false, name);
    const std::string tmpz = sz.str();
    name = tmpz.c_str();;
    cloud_viewer_.removeShape(name);
    cloud_viewer_.addArrow (pcl::PointXYZ(stair.initial_point.x+stair.stair_dir.col(2)(0)*1, stair.initial_point.y+stair.stair_dir.col(2)(1)*1, stair.initial_point.z+stair.stair_dir.col(2)(2)*1), stair.initial_point, 0.0f, 0.0f, 1.0f, false, name);
}

void ViewerStair::drawStairAxis (Stair stair, std::string stair_type, Eigen::Affine3d pose) {
    Eigen::Vector3f initial_point;
    Eigen::Vector3f point_x;
    Eigen::Vector3f point_y;
    Eigen::Vector3f point_z;

    pcl::transformPoint(stair.initial_point.getVector3fMap(),initial_point,pose.cast<float>());
    pcl::transformPoint(Eigen::Vector3f(stair.initial_point.x+stair.stair_dir.col(0)(0)*1, stair.initial_point.y+stair.stair_dir.col(0)(1)*1, stair.initial_point.z+stair.stair_dir.col(0)(2)*1),point_x,pose.cast<float>());
    pcl::transformPoint(Eigen::Vector3f(stair.initial_point.x+stair.stair_dir.col(1)(0)*1, stair.initial_point.y+stair.stair_dir.col(1)(1)*1, stair.initial_point.z+stair.stair_dir.col(1)(2)*1),point_y,pose.cast<float>());
    pcl::transformPoint(Eigen::Vector3f(stair.initial_point.x+stair.stair_dir.col(2)(0)*1, stair.initial_point.y+stair.stair_dir.col(2)(1)*1, stair.initial_point.z+stair.stair_dir.col(2)(2)*1),point_z,pose.cast<float>());

    std::stringstream sx,sy,sz;
    sx << stair_type << "x";
    sy << stair_type << "y";
    sz << stair_type << "z";
    const std::string tmpx = sx.str();
    const char* name = tmpx.c_str();
    cloud_viewer_.removeShape(name);
    cloud_viewer_.addArrow (pcl::PointXYZ(point_x(0),point_x(1),point_x(2)), pcl::PointXYZ(initial_point(0),initial_point(1),initial_point(2)), 1.0f, 0.0f, 0.0f, false, name);
    const std::string tmpy = sy.str();
    name = tmpy.c_str();
    cloud_viewer_.removeShape(name);
    cloud_viewer_.addArrow (pcl::PointXYZ(point_y(0),point_y(1),point_y(2)), pcl::PointXYZ(initial_point(0),initial_point(1),initial_point(2)), 1.0f, 0.0f, 0.0f, false, name);
    const std::string tmpz = sz.str();
    name = tmpz.c_str();;
    cloud_viewer_.removeShape(name);
    cloud_viewer_.addArrow (pcl::PointXYZ(point_z(0),point_z(1),point_z(2)), pcl::PointXYZ(initial_point(0),initial_point(1),initial_point(2)), 1.0f, 0.0f, 0.0f, false, name);


}

void ViewerStair::drawStairs (Stair stair, std::string stair_type) {
    for (int Q = 1; Q<stair.vLevels.size(); Q++) {
        std::stringstream ss;
        ss << stair_type << Q + 10;
        const std::string tmp = ss.str();
        const char* name = tmp.c_str();

        this->drawRectangle(stair.vLevels[Q].vertices,0,0,1,name);
        // this->drawRectangle(stair.vOrientedLevels[Q].vertices,0,0,1,name);
    }
}

void ViewerStair::drawStairs (Stair stair, std::string stair_type, Eigen::Affine3d pose) {
    for (int Q = 1; Q<stair.vLevels.size(); Q++) {
        std::stringstream ss;
        ss << stair_type << Q+10;
        const std::string tmp = ss.str();
        const char* name = tmp.c_str();
        pcl::PointCloud<pcl::PointXYZ>::Ptr vertex_to_draw (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*stair.vLevels[Q].vertices,*vertex_to_draw,pose);
        this->drawRectangle(vertex_to_draw,0,0,1,name);
        // this->drawRectangle(stair.vLevels[Q].vertices,0,0,1,name);
        // this->drawRectangle(stair.vOrientedLevels[Q].vertices,0,0,1,name);
    }
}

void ViewerStair::drawRisers (Stair stair, std::string stair_type) {

    for (int Q = 1; Q<stair.vLevels.size(); Q++) {
        std::stringstream ss;
        ss << stair_type << Q;
        const std::string tmp = ss.str();
        const char* name = tmp.c_str();

        //~ this->drawRectangle(stair.vLevels[Q].vertices,0,0,1,name);
        if (stair.vRisers[Q].vertices->points.size()>0)
            this->drawRectangle(stair.vRisers[Q].vertices,1,1,0,name);
    }
}

void ViewerStair::drawRisers (Stair stair, std::string stair_type, Eigen::Affine3d pose) {
    for (int Q = 1; Q<stair.vRisers.size(); Q++) {
        std::stringstream ss;
        ss << stair_type << Q;
        const std::string tmp = ss.str();
        const char* name = tmp.c_str();
        pcl::PointCloud<pcl::PointXYZ>::Ptr riser_vertex_to_draw (new pcl::PointCloud<pcl::PointXYZ>);

        pcl::transformPointCloud(*stair.vRisers[Q].vertices,*riser_vertex_to_draw,pose);
        if (stair.vRisers[Q].vertices->points.size()>0)
            this->drawRectangle(riser_vertex_to_draw,1,1,0,name);
    }
}

void ViewerStair::drawFullStairUntil (Stair stair, int level, Eigen::Affine3d s2p) {

    for (int Q = 1; Q<level; Q++)  {
        pcl::PointCloud<pcl::PointXYZ>::Ptr vertices (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr riser_vertices (new pcl::PointCloud<pcl::PointXYZ>);


        pcl::PointXYZ pt0(-stair.step_width/2,stair.step_height*(Q-1),stair.step_length*Q);
        pcl::PointXYZ pt1(stair.step_width/2,stair.step_height*(Q-1),stair.step_length*Q);
        pcl::PointXYZ pt2(stair.step_width/2,stair.step_height*(Q-1),stair.step_length*(Q-1));
        pcl::PointXYZ pt3(-stair.step_width/2,stair.step_height*(Q-1),stair.step_length*(Q-1));
        pcl::PointXYZ pt4(0,stair.step_height*(Q-1),(stair.step_length*(Q)+stair.step_length*(Q-1))/2);

        vertices->points.push_back(pt0);vertices->points.push_back(pt1);vertices->points.push_back(pt2);
        vertices->points.push_back(pt3);vertices->points.push_back(pt4);

        pt0 = pcl::PointXYZ(-stair.step_width/2,stair.step_height*(Q-1),stair.step_length*(Q-1));
        pt1 = pcl::PointXYZ(stair.step_width/2,stair.step_height*(Q-1),stair.step_length*(Q-1));
        pt2 = pcl::PointXYZ(stair.step_width/2,stair.step_height*(Q-2),stair.step_length*(Q-1));
        pt3 = pcl::PointXYZ(-stair.step_width/2,stair.step_height*(Q-2),stair.step_length*(Q-1));
        pt4 = pcl::PointXYZ(0,(stair.step_height*(Q-2)+stair.step_height*(Q-1))/2,stair.step_length*(Q-1));

        riser_vertices->points.push_back(pt0);riser_vertices->points.push_back(pt1);riser_vertices->points.push_back(pt2);
        riser_vertices->points.push_back(pt3);riser_vertices->points.push_back(pt4);


        std::stringstream ss;
        ss << "step_" << Q;
        const std::string tmp = ss.str();
        const char* name = tmp.c_str();
        std::stringstream ssr;
        ssr << "riser_" << Q;
        const std::string rtmp = ssr.str();
        const char* rname = rtmp.c_str();

        std::stringstream text;
        text << "step_text_" << Q;
        cloud_viewer_.removeText3D(text.str());
        std::stringstream stepstring;
        stepstring << "Step " << Q;
        Eigen::Vector3f point_to_draw2s(vertices->points[4].x,vertices->points[4].y + 0.20f, vertices->points[4].z);
        Eigen::Vector3f point_to_draw2c;
        pcl::transformPoint(point_to_draw2s,point_to_draw2c,s2p.cast<float>());
        //    cloud_viewer_.addText3D (stepstring.str(), getPointXYZ(point_to_draw2c), 0.05, 1.0, 1.0, 1.0, text.str());
        cloud_viewer_.addText3D (stepstring.str(), pcl::PointXYZ(point_to_draw2c(0),point_to_draw2c(1),point_to_draw2c(2)), 0.05, 1.0, 1.0, 1.0, text.str());


        pcl::transformPointCloud(*vertices,*vertices,s2p);
        pcl::transformPointCloud(*riser_vertices,*riser_vertices,s2p);

        this->drawRectangle(vertices,0,0,1,name);
        this->drawRectangle(riser_vertices,1,1,0,rname);
    }
}

void ViewerStair::drawFullAscendingStairUntil (Stair stair, int level, Eigen::Affine3d s2p) {

    for (int Q = 1; Q<level; Q++) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr vertices (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr riser_vertices (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr left_vertices (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr right_vertices (new pcl::PointCloud<pcl::PointXYZ>);

        pcl::PointXYZ pt0(-stair.step_width/2,	stair.step_height*(Q-1),	stair.step_length*Q);
        pcl::PointXYZ pt1(stair.step_width/2,	stair.step_height*(Q-1),	stair.step_length*Q);
        pcl::PointXYZ pt2(stair.step_width/2,	stair.step_height*(Q-1),	stair.step_length*(Q-1));
        pcl::PointXYZ pt3(-stair.step_width/2,	stair.step_height*(Q-1),	stair.step_length*(Q-1));
        pcl::PointXYZ pt4(0,					stair.step_height*(Q-1),	(stair.step_length*(Q)+stair.step_length*(Q-1))/2);

        vertices->points.push_back(pt0);vertices->points.push_back(pt1);vertices->points.push_back(pt2);
        vertices->points.push_back(pt3);vertices->points.push_back(pt4);

        pt0 = pcl::PointXYZ(-stair.step_width/2,	stair.step_height*(Q-1),								stair.step_length*(Q-1));
        pt1 = pcl::PointXYZ(stair.step_width/2,		stair.step_height*(Q-1),								stair.step_length*(Q-1));
        pt2 = pcl::PointXYZ(stair.step_width/2,		stair.step_height*(Q-2),								stair.step_length*(Q-1));
        pt3 = pcl::PointXYZ(-stair.step_width/2,	stair.step_height*(Q-2),								stair.step_length*(Q-1));
        pt4 = pcl::PointXYZ(0,						(stair.step_height*(Q-2)+stair.step_height*(Q-1))/2,	stair.step_length*(Q-1));

        riser_vertices->points.push_back(pt0);riser_vertices->points.push_back(pt1);riser_vertices->points.push_back(pt2);
        riser_vertices->points.push_back(pt3);riser_vertices->points.push_back(pt4);

        pt0 = pcl::PointXYZ(stair.step_width/2,	stair.step_height*(Q-1),							stair.step_length*(Q));
        pt1 = pcl::PointXYZ(stair.step_width/2,	stair.step_height*(Q-1),							stair.step_length*(Q-1));
        pt2 = pcl::PointXYZ(stair.step_width/2,	-stair.step_height,									stair.step_length*(Q-1));
        pt3 = pcl::PointXYZ(stair.step_width/2,	-stair.step_height,									stair.step_length*(Q));
        pt4 = pcl::PointXYZ(stair.step_width/2,	(stair.step_height*(Q-1) - stair.step_height)/2,	(stair.step_length*(Q) + stair.step_length*(Q-1))/2);

        left_vertices->points.push_back(pt0);left_vertices->points.push_back(pt1);left_vertices->points.push_back(pt2);
        left_vertices->points.push_back(pt3);left_vertices->points.push_back(pt4);

        pt0 = pcl::PointXYZ(-stair.step_width/2,	stair.step_height*(Q-1),							stair.step_length*(Q));
        pt1 = pcl::PointXYZ(-stair.step_width/2,	stair.step_height*(Q-1),							stair.step_length*(Q-1));
        pt2 = pcl::PointXYZ(-stair.step_width/2,	-stair.step_height,									stair.step_length*(Q-1));
        pt3 = pcl::PointXYZ(-stair.step_width/2,	-stair.step_height,									stair.step_length*(Q));
        pt4 = pcl::PointXYZ(-stair.step_width/2,	(stair.step_height*(Q-1) - stair.step_height)/2,	(stair.step_length*(Q) + stair.step_length*(Q-1))/2);

        right_vertices->points.push_back(pt0);right_vertices->points.push_back(pt1);right_vertices->points.push_back(pt2);
        right_vertices->points.push_back(pt3);right_vertices->points.push_back(pt4);


        std::stringstream ss;
        ss << "stepup_" << Q;
        const std::string tmp = ss.str();
        const char* name = tmp.c_str();
        std::stringstream ssr;
        ssr << "riserup_" << Q;
        const std::string rtmp = ssr.str();
        const char* rname = rtmp.c_str();
        std::stringstream sslp;
        sslp << "leftup_" << Q;
        const std::string lptmp = sslp.str();
        const char* lpname = lptmp.c_str();
        std::stringstream ssrp;
        ssrp << "rightup_" << Q;
        const std::string rptmp = ssrp.str();
        const char* rpname = rptmp.c_str();

        std::stringstream text;
        text << "step_textup_" << Q;
        cloud_viewer_.removeText3D(text.str());
        std::stringstream stepstring;
        stepstring << "Step " << Q;
        Eigen::Vector3f point_to_draw2s(vertices->points[4].x,vertices->points[4].y + 0.10f, vertices->points[4].z);
        Eigen::Vector3f point_to_draw2c;
        pcl::transformPoint(point_to_draw2s,point_to_draw2c,s2p.cast<float>());
        cloud_viewer_.addText3D (stepstring.str(), pcl::PointXYZ(point_to_draw2c(0),point_to_draw2c(1),point_to_draw2c(2)), 0.05, 1.0, 1.0, 1.0, text.str());
        pcl::transformPointCloud(*vertices,*vertices,s2p);
        pcl::transformPointCloud(*riser_vertices,*riser_vertices,s2p);
        pcl::transformPointCloud(*left_vertices,*left_vertices,s2p);
        pcl::transformPointCloud(*right_vertices,*right_vertices,s2p);

        this->drawRectangle(vertices,0,0,1,name);
        this->drawRectangle(riser_vertices,1,1,0,rname);
        this->drawRectangle(left_vertices,0,1,0,lpname);
        this->drawRectangle(right_vertices,0,1,0,rpname);



    }
}

void ViewerStair::drawFullAscendingStairUntil (Stair stair, int level, Eigen::Affine3d s2p, Eigen::Affine3d pose) {

    for (int Q = 1; Q<level; Q++) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr vertices (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr riser_vertices (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr left_vertices (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr right_vertices (new pcl::PointCloud<pcl::PointXYZ>);

        pcl::PointXYZ pt0(-stair.step_width/2,	stair.step_height*(Q-1),	stair.step_length*Q);
        pcl::PointXYZ pt1(stair.step_width/2,	stair.step_height*(Q-1),	stair.step_length*Q);
        pcl::PointXYZ pt2(stair.step_width/2,	stair.step_height*(Q-1),	stair.step_length*(Q-1));
        pcl::PointXYZ pt3(-stair.step_width/2,	stair.step_height*(Q-1),	stair.step_length*(Q-1));
        pcl::PointXYZ pt4(0,					stair.step_height*(Q-1),	(stair.step_length*(Q)+stair.step_length*(Q-1))/2);

        vertices->points.push_back(pt0);vertices->points.push_back(pt1);vertices->points.push_back(pt2);
        vertices->points.push_back(pt3);vertices->points.push_back(pt4);

        pt0 = pcl::PointXYZ(-stair.step_width/2,	stair.step_height*(Q-1),								stair.step_length*(Q-1));
        pt1 = pcl::PointXYZ(stair.step_width/2,		stair.step_height*(Q-1),								stair.step_length*(Q-1));
        pt2 = pcl::PointXYZ(stair.step_width/2,		stair.step_height*(Q-2),								stair.step_length*(Q-1));
        pt3 = pcl::PointXYZ(-stair.step_width/2,	stair.step_height*(Q-2),								stair.step_length*(Q-1));
        pt4 = pcl::PointXYZ(0,						(stair.step_height*(Q-2)+stair.step_height*(Q-1))/2,	stair.step_length*(Q-1));

        riser_vertices->points.push_back(pt0);riser_vertices->points.push_back(pt1);riser_vertices->points.push_back(pt2);
        riser_vertices->points.push_back(pt3);riser_vertices->points.push_back(pt4);

        pt0 = pcl::PointXYZ(stair.step_width/2,	stair.step_height*(Q-1),							stair.step_length*(Q));
        pt1 = pcl::PointXYZ(stair.step_width/2,	stair.step_height*(Q-1),							stair.step_length*(Q-1));
        pt2 = pcl::PointXYZ(stair.step_width/2,	-stair.step_height,									stair.step_length*(Q-1));
        pt3 = pcl::PointXYZ(stair.step_width/2,	-stair.step_height,									stair.step_length*(Q));
        pt4 = pcl::PointXYZ(stair.step_width/2,	(stair.step_height*(Q-1) - stair.step_height)/2,	(stair.step_length*(Q) + stair.step_length*(Q-1))/2);

        left_vertices->points.push_back(pt0);left_vertices->points.push_back(pt1);left_vertices->points.push_back(pt2);
        left_vertices->points.push_back(pt3);left_vertices->points.push_back(pt4);

        pt0 = pcl::PointXYZ(-stair.step_width/2,	stair.step_height*(Q-1),							stair.step_length*(Q));
        pt1 = pcl::PointXYZ(-stair.step_width/2,	stair.step_height*(Q-1),							stair.step_length*(Q-1));
        pt2 = pcl::PointXYZ(-stair.step_width/2,	-stair.step_height,									stair.step_length*(Q-1));
        pt3 = pcl::PointXYZ(-stair.step_width/2,	-stair.step_height,									stair.step_length*(Q));
        pt4 = pcl::PointXYZ(-stair.step_width/2,	(stair.step_height*(Q-1) - stair.step_height)/2,	(stair.step_length*(Q) + stair.step_length*(Q-1))/2);

        right_vertices->points.push_back(pt0);right_vertices->points.push_back(pt1);right_vertices->points.push_back(pt2);
        right_vertices->points.push_back(pt3);right_vertices->points.push_back(pt4);


        std::stringstream ss;
        ss << "stepup_" << Q;
        const std::string tmp = ss.str();
        const char* name = tmp.c_str();
        std::stringstream ssr;
        ssr << "riserup_" << Q;
        const std::string rtmp = ssr.str();
        const char* rname = rtmp.c_str();
        std::stringstream sslp;
        sslp << "leftup_" << Q;
        const std::string lptmp = sslp.str();
        const char* lpname = lptmp.c_str();
        std::stringstream ssrp;
        ssrp << "rightup_" << Q;
        const std::string rptmp = ssrp.str();
        const char* rpname = rptmp.c_str();

        std::stringstream text;
        text << "step_textup_" << Q;
        cloud_viewer_.removeText3D(text.str());
        std::stringstream stepstring;
        stepstring << "Step " << Q;
        Eigen::Vector3f point_to_draw2s(vertices->points[4].x,vertices->points[4].y + 0.10f, vertices->points[4].z);
        Eigen::Vector3f point_to_draw2c;
        pcl::transformPoint(point_to_draw2s,point_to_draw2c,s2p.cast<float>());
        cloud_viewer_.addText3D (stepstring.str(), pcl::PointXYZ(point_to_draw2c(0),point_to_draw2c(1),point_to_draw2c(2)), 0.05, 1.0, 1.0, 1.0, text.str());
        pcl::transformPointCloud(*vertices,*vertices,pose*s2p);
        pcl::transformPointCloud(*riser_vertices,*riser_vertices,pose*s2p);
        this->drawRectangle(vertices,0,0,1,name);
        this->drawRectangle(riser_vertices,1,1,0,rname);

        pcl::transformPointCloud(*left_vertices,*left_vertices,pose*s2p);
        pcl::transformPointCloud(*right_vertices,*right_vertices,pose*s2p);
        this->drawRectangle(left_vertices,0,1,0,lpname);
        this->drawRectangle(right_vertices,0,1,0,rpname);


    }
}

void ViewerStair::drawFullDescendingStairUntil (Stair stair, int level, Eigen::Affine3d s2p){

    for (int Q = 1; Q<level; Q++) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr vertices (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr riser_vertices (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr left_vertices (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr right_vertices (new pcl::PointCloud<pcl::PointXYZ>);

        pcl::PointXYZ pt0(-stair.step_width/2,-stair.step_height*(Q-1),stair.step_length*Q);
        pcl::PointXYZ pt1(stair.step_width/2,-stair.step_height*(Q-1),stair.step_length*Q);
        pcl::PointXYZ pt2(stair.step_width/2,-stair.step_height*(Q-1),stair.step_length*(Q-1));
        pcl::PointXYZ pt3(-stair.step_width/2,-stair.step_height*(Q-1),stair.step_length*(Q-1));
        pcl::PointXYZ pt4(0,-stair.step_height*(Q-1),(stair.step_length*(Q)+stair.step_length*(Q-1))/2);

        vertices->points.push_back(pt0);vertices->points.push_back(pt1);vertices->points.push_back(pt2);
        vertices->points.push_back(pt3);vertices->points.push_back(pt4);

        pt0 = pcl::PointXYZ(-stair.step_width/2,-stair.step_height*(Q-2),stair.step_length*(Q-1));
        pt1 = pcl::PointXYZ(stair.step_width/2,-stair.step_height*(Q-2),stair.step_length*(Q-1));
        pt2 = pcl::PointXYZ(stair.step_width/2,-stair.step_height*(Q-1),stair.step_length*(Q-1));
        pt3 = pcl::PointXYZ(-stair.step_width/2,-stair.step_height*(Q-1),stair.step_length*(Q-1));
        pt4 = pcl::PointXYZ(0,-(stair.step_height*(Q-1)+stair.step_height*(Q-2))/2,stair.step_length*(Q-1));

        riser_vertices->points.push_back(pt0);riser_vertices->points.push_back(pt1);riser_vertices->points.push_back(pt2);
        riser_vertices->points.push_back(pt3);riser_vertices->points.push_back(pt4);

        pt0 = pcl::PointXYZ(stair.step_width/2,	-stair.step_height*(Q-1),							stair.step_length*(Q));
        pt1 = pcl::PointXYZ(stair.step_width/2,	-stair.step_height*(Q-1),							stair.step_length*(Q-1));
        pt2 = pcl::PointXYZ(stair.step_width/2,	stair.step_height,									stair.step_length*(Q-1));
        pt3 = pcl::PointXYZ(stair.step_width/2,	stair.step_height,									stair.step_length*(Q));
        pt4 = pcl::PointXYZ(stair.step_width/2,	(-stair.step_height*(Q-1) + stair.step_height)/2,	(stair.step_length*(Q) + stair.step_length*(Q-1))/2);

        left_vertices->points.push_back(pt0);left_vertices->points.push_back(pt1);left_vertices->points.push_back(pt2);
        left_vertices->points.push_back(pt3);left_vertices->points.push_back(pt4);

        pt0 = pcl::PointXYZ(-stair.step_width/2,	-stair.step_height*(Q-1),							stair.step_length*(Q));
        pt1 = pcl::PointXYZ(-stair.step_width/2,	-stair.step_height*(Q-1),							stair.step_length*(Q-1));
        pt2 = pcl::PointXYZ(-stair.step_width/2,	stair.step_height,									stair.step_length*(Q-1));
        pt3 = pcl::PointXYZ(-stair.step_width/2,	stair.step_height,									stair.step_length*(Q));
        pt4 = pcl::PointXYZ(-stair.step_width/2,	(-stair.step_height*(Q-1) + stair.step_height)/2,	(stair.step_length*(Q) + stair.step_length*(Q-1))/2);

        right_vertices->points.push_back(pt0);right_vertices->points.push_back(pt1);right_vertices->points.push_back(pt2);
        right_vertices->points.push_back(pt3);right_vertices->points.push_back(pt4);


        std::stringstream ss;
        ss << "stepdown_" << Q;
        const std::string tmp = ss.str();
        const char* name = tmp.c_str();
        std::stringstream ssr;
        ssr << "riserdown_" << Q;
        const std::string rtmp = ssr.str();
        const char* rname = rtmp.c_str();
        std::stringstream sslp;
        sslp << "leftdown_" << Q;
        const std::string lptmp = sslp.str();
        const char* lpname = lptmp.c_str();
        std::stringstream ssrp;
        ssrp << "rightdown_" << Q;
        const std::string rptmp = ssrp.str();
        const char* rpname = rptmp.c_str();

        std::stringstream text;
        text << "step_textdown_" << Q;
        cloud_viewer_.removeText3D(text.str());
        std::stringstream stepstring;
        stepstring << "Step -" << Q;
        Eigen::Vector3f point_to_draw2s(vertices->points[4].x,vertices->points[4].y + 0.20f, vertices->points[4].z);
        Eigen::Vector3f point_to_draw2c;
        pcl::transformPoint(point_to_draw2s,point_to_draw2c,s2p.cast<float>());
        cloud_viewer_.addText3D (stepstring.str(), pcl::PointXYZ(point_to_draw2c(0),point_to_draw2c(1),point_to_draw2c(2)), 0.05, 1.0, 1.0, 1.0, text.str());
        pcl::transformPointCloud(*vertices,*vertices,s2p);
        pcl::transformPointCloud(*riser_vertices,*riser_vertices,s2p);
        pcl::transformPointCloud(*left_vertices,*left_vertices,s2p);
        pcl::transformPointCloud(*right_vertices,*right_vertices,s2p);

        this->drawRectangle(vertices,0,0,1,name);
        this->drawRectangle(riser_vertices,1,1,0,rname);
        this->drawRectangle(left_vertices,0,1,0,lpname);
        this->drawRectangle(right_vertices,0,1,0,rpname);
    }
}

void ViewerStair::addStairsText (Eigen::Affine3d i2s, Eigen::Affine3d f2c, std::string type) {

    Eigen::Affine3d f2s = i2s * f2c;
    double x = f2s.translation()[0];
    double z = f2s.translation()[2];

    std::stringstream ss;
    if (type == "up"){
        ss << "Ascending stairs at " << sqrt(x*x+z*z) << "m";
        cloud_viewer_.addText (ss.str(), 50, 50, 20, 1.0, 1.0, 1.0, "uptext");
    }
    else{
        ss << "Descending stairs at " << sqrt(x*x+z*z) << "m";
        cloud_viewer_.addText (ss.str(), 50, 100, 20, 1.0, 1.0, 1.0, "downtext");
    }
}
