#include "../../include/Utility/Reader.h"
#include "../../include/Common/Type.h"

#include <fstream>
#include <sstream>
#include <string.h>

#include <opencv2/opencv.hpp>

using namespace std;

namespace Utility{
    Matrix<float,IMG_H,IMG_W> Reader::readinDepth(int sampleNo,int imgNo){

        char depth_file_name[100];
        sprintf(depth_file_name,"%s/%s/scene_%02d_%04d.depth",Common::dataPath,Common::gtSubfolder,sampleNo,imgNo);

        ifstream file;

        file.open(depth_file_name); 

        Matrix<float,IMG_H,IMG_W> depth = Matrix<float,IMG_H,IMG_W>::Zero();

        for (int i=0;i<IMG_H;i++){
            for (int j=0;j<IMG_W;j++){
                file >> depth(i,j);
            }
        }

        return depth;
    };

    Matrix<float,3,4> Reader::readinPose(int sampleNo,int imgNo){

        char text_file_name[100];
        sprintf(text_file_name,"%s/%s/scene_%02d_%04d.txt",Common::dataPath,Common::gtSubfolder,sampleNo,imgNo);

        cout << "text_file_name = " << text_file_name << endl;

        ifstream cam_pars_file(text_file_name);

        char readlinedata[300];

        Common::float4 direction;
        Common::float4 upvector;
        float posvector[3];

        while(1){
            cam_pars_file.getline(readlinedata,300);

            if ( cam_pars_file.eof())
                break;
            
            istringstream iss;

            if ( strstr(readlinedata,"cam_dir")!= NULL){

                string cam_dir_str(readlinedata);

                cam_dir_str = cam_dir_str.substr(cam_dir_str.find("= [")+3);
                cam_dir_str = cam_dir_str.substr(0,cam_dir_str.find("]"));

                iss.str(cam_dir_str);
                iss >> direction.x ;
                iss.ignore(1,',');
                iss >> direction.y ;
                iss.ignore(1,',') ;
                iss >> direction.z;
                iss.ignore(1,',');
                // cout << direction.x<< ", "<< direction.y << ", "<< direction.z << endl;
                direction.w = 0.0f;

            }

            if ( strstr(readlinedata,"cam_up")!= NULL){

                string cam_up_str(readlinedata);

                cam_up_str = cam_up_str.substr(cam_up_str.find("= [")+3);
                cam_up_str = cam_up_str.substr(0,cam_up_str.find("]"));

                iss.str(cam_up_str);
                iss >> upvector.x ;
                iss.ignore(1,',');
                iss >> upvector.y ;
                iss.ignore(1,',');
                iss >> upvector.z ;
                iss.ignore(1,',');

                upvector.w = 0.0f;
            }

            if ( strstr(readlinedata,"cam_pos")!= NULL){
                // cout<< "cam_pos is present!"<<endl;

                string cam_pos_str(readlinedata);

                cam_pos_str = cam_pos_str.substr(cam_pos_str.find("= [")+3);
                cam_pos_str = cam_pos_str.substr(0,cam_pos_str.find("]"));

                // cout << "cam pose str = " << endl;
                // cout << cam_pos_str << endl;

                iss.str(cam_pos_str);
                iss >> posvector[0] ;
                iss.ignore(1,',');
                iss >> posvector[1] ;
                iss.ignore(1,',');
                iss >> posvector[2] ;
                iss.ignore(1,',');
                // cout << posvector[0]<< ", "<< posvector[1] << ", "<< posvector[2] << endl;

            }

        }

        /// z = dir / norm(dir)
        cv::Vec<float,3> z;
        z[0] = direction.x;
        z[1] = direction.y;
        z[2] = direction.z;
        cv::normalize(z);
    //    cout << " z = " << z << endl;

        /// x = cross(cam_up, z)
        cv::Vec<float,3> x;
        x[0] =  upvector.y * z[2] - upvector.z * z[1];
        x[1] =  upvector.z * z[0] - upvector.x * z[2];
        x[2] =  upvector.x * z[1] - upvector.y * z[0];

        normalize(x);
    //    cout << " x = " << x << endl;

        /// y = cross(z,x)
        cv::Vec<float,3> y;
        y[0] =  z[1] * x[2] - z[2] * x[1];
        y[1] =  z[2] * x[0] - z[0] * x[2];
        y[2] =  z[0] * x[1] - z[1] * x[0];

    //    cout << " y = " << y << endl;

        Matrix<float,3,4> R = Matrix<float,3,4>::Zero();

        R(0,0) = x[0];
        R(1,0) = x[1];
        R(2,0) = x[2];

        R(0,1) = y[0];
        R(1,1) = y[1];
        R(2,1) = y[2];

        R(0,2) = z[0];
        R(1,2) = z[1];
        R(2,2) = z[2];

        R(0,3) = posvector[0];
        R(1,3) = posvector[1];
        R(2,3) = posvector[2];

        cout << "R = " << endl << R << endl;

        cout << "finished" << endl;

        return R;
    }

    cv::Mat Reader::readinImg(int sampleNo,int imgNo){

        char img_file_name[100];
        sprintf(img_file_name,"%s/%s/scene_%02d_%04d.png",Common::dataPath,Common::imgSubfolder,sampleNo,imgNo);

        cout << "img_file_name = " << img_file_name << endl;

        return cv::imread(img_file_name,0);
    };
    
}