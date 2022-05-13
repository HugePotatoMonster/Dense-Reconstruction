#include "../../include/Misc/msAuxiliaryUtils.h"
#include "../../include/Common/cmMath.h"
namespace Misc {
	void AuxiliaryUtils::msParseExtrinsic(std::string file, OUT_ARG Common::Camera::MonocularCameraExtrinsic* camEx) {
		// Ported from Official Implementation
		// https://www.doc.ic.ac.uk/~ahanda/HighFrameRateTracking/computeTpov_cam.cpp
		std::ifstream fs(file);
		char linedata[300];
        f64 direction[4] = { 0,0,0,0 };
        f64 up[4];
        f64 posvector[4];
		while (1) {
			fs.getline(linedata, 300);
			if (fs.eof()) {
				break;
			}
            std::istringstream iss;
            if (strstr(linedata, "cam_dir") != NULL) {
                std::string cam_dir_str(linedata);
                cam_dir_str = cam_dir_str.substr(cam_dir_str.find("= [") + 3);
                cam_dir_str = cam_dir_str.substr(0, cam_dir_str.find("]"));
                iss.str(cam_dir_str);
                iss >> direction[0];
                iss.ignore(1, ',');
                iss >> direction[1];
                iss.ignore(1, ',');
                iss >> direction[2];
                iss.ignore(1, ',');
                direction[3] = 0.0f;
                f64 len = len_vec3(direction[0], direction[1], direction[2]);
                for (i32 i = 0; i < 3; i++) {
                    norm_vec3(direction[i], len);
                }
            }
            if (strstr(linedata, "cam_up") != NULL)
            {
                std::string cam_up_str(linedata);
                cam_up_str = cam_up_str.substr(cam_up_str.find("= [") + 3);
                cam_up_str = cam_up_str.substr(0, cam_up_str.find("]"));
                iss.str(cam_up_str);
                iss >> up[0];
                iss.ignore(1, ',');
                iss >> up[1];
                iss.ignore(1, ',');
                iss >> up[2];
                iss.ignore(1, ',');
                up[3] = 0.0;
                f64 len = len_vec3(up[0], up[1], up[2]);
                for (i32 i = 0; i < 3; i++) {
                    norm_vec3(up[i], len);
                }

            }
            if (strstr(linedata, "cam_pos") != NULL)
            {
                std::string cam_pos_str(linedata);
                cam_pos_str = cam_pos_str.substr(cam_pos_str.find("= [") + 3);
                cam_pos_str = cam_pos_str.substr(0, cam_pos_str.find("]"));
                iss.str(cam_pos_str);
                iss >> posvector[0];
                iss.ignore(1, ',');
                iss >> posvector[1];
                iss.ignore(1, ',');
                iss >> posvector[2];
                iss.ignore(1, ',');
                f64 len = len_vec3(posvector[0], posvector[1], posvector[2]);
                //posvector[1] = -posvector[1];
                for (i32 i = 0; i < 3; i++) {
                    posvector[i] = -posvector[i];
                    camEx->t[i] = posvector[i];
                    norm_vec3(posvector[i], len);
                }
            }
		}
        Common::Math::Vec3 z = { {direction[0],direction[1],direction[2]} };
        Common::Math::Vec3 x;
        x.a[0] = up[1] * z.a[2] - up[2] * z.a[1];
        x.a[1] = up[2] * z.a[0] - up[0] * z.a[2];
        x.a[2] = up[0] * z.a[1] - up[1] * z.a[0];
        x.normalize();
        Common::Math::Vec3 y;
        y.a[0] = z.a[1] * x.a[2] - z.a[2] * x.a[1];
        y.a[1] = z.a[2] * x.a[0] - z.a[0] * x.a[2];
        y.a[2] = z.a[0] * x.a[1] - z.a[1] * x.a[0];
        y.normalize();
        
        cv::Mat camExR1 = cv::Mat_<f64>(3,3);
        for (i32 i = 0; i < 3; i++) {
            get_cvmat(camExR1,i,0) = x.a[i];
            get_cvmat(camExR1,i,1) = y.a[i];
            get_cvmat(camExR1,i,2) = z.a[i];
        }
        camExR1 = camExR1.inv();
        for (i32 i = 0; i < 3; i++) {
            for(i32 j=0;j<3;j++){
                camEx->r[i][j] = get_cvmat(camExR1,i,j);
            }
        }
	}
    void AuxiliaryUtils::msParseIntrinsic(std::string file, OUT_ARG Common::Camera::MonocularCameraIntrinsic* camIn) {
        // Ported from Official Implementation
        // https://www.doc.ic.ac.uk/~ahanda/HighFrameRateTracking/computeTpov_cam.cpp
        std::ifstream fs(file);
        char linedata[300];
        Common::Math::Vec3 dir;
        Common::Math::Vec3 right;
        Common::Math::Vec3 up;
        while (1) {
            fs.getline(linedata, 300);
            if (fs.eof()) {
                break;
            }
            std::istringstream iss;
            if (strstr(linedata, "cam_dir") != NULL) {
                std::string cam_dir_str(linedata);
                cam_dir_str = cam_dir_str.substr(cam_dir_str.find("= [") + 3);
                cam_dir_str = cam_dir_str.substr(0, cam_dir_str.find("]"));
                iss.str(cam_dir_str);
                iss >> dir.a[0];
                iss.ignore(1, ',');
                iss >> dir.a[1];
                iss.ignore(1, ',');
                iss >> dir.a[2];
                iss.ignore(1, ',');
            }
            if (strstr(linedata, "cam_right") != NULL) {
                std::string cam_dir_str(linedata);
                cam_dir_str = cam_dir_str.substr(cam_dir_str.find("= [") + 3);
                cam_dir_str = cam_dir_str.substr(0, cam_dir_str.find("]"));
                iss.str(cam_dir_str);
                iss >> right.a[0];
                iss.ignore(1, ',');
                iss >> right.a[1];
                iss.ignore(1, ',');
                iss >> right.a[2];
                iss.ignore(1, ',');
            }
            if (strstr(linedata, "cam_up") != NULL) {
                std::string cam_dir_str(linedata);
                cam_dir_str = cam_dir_str.substr(cam_dir_str.find("= [") + 3);
                cam_dir_str = cam_dir_str.substr(0, cam_dir_str.find("]"));
                iss.str(cam_dir_str);
                iss >> up.a[0];
                iss.ignore(1, ',');
                iss >> up.a[1];
                iss.ignore(1, ',');
                iss >> up.a[2];
                iss.ignore(1, ',');
            }
        }
        std::cout<<"dir"<<dir.a[0]<<","<<dir.a[1]<<","<<dir.a[2]<<std::endl;
        f64 focal;
        dir.dist(&focal);
        f64 rl, us, aspect, angle;
        right.dist(&rl);
        up.dist(&us);
        aspect = rl / us;
        angle = atan(rl / 2.0 / focal) * 2;
        std::cout<<"focal="<<focal<<",angle="<<angle<<",aspect="<<aspect<<std::endl;
        f64 width = 640, height = 480;
        f64 psx = 2.0 * focal * tan(0.5 * angle) / width;
        f64 psy = 2.0 * focal * tan(0.5 * angle) / aspect / height;
        psx /= focal;
        psy /= focal;

        camIn->dx = 1;
        camIn->dx = 1;
        camIn->fx = 1.0 / psx;
        camIn->fy = -1.0 / psy;
        camIn->cx = (width + 0.0) / 2.0;
        camIn->cy = (height + 0.0) / 2.0;
    }
    
}