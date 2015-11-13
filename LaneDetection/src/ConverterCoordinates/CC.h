#pragma once

#include <math.h>

#if _MSC_VER == 1700
inline double atanh(double x) 
{
	return (log(1+x) - log(1-x))/2;
}
#endif

#define CC_PI 3.1415926535897932384626433832795
/*
Mathematic computations for Convertering Coordinates
*/
namespace CC {

	/*
	It's the rotation of 2d points (not system rotation) so in the same coordinate system. 
	Counterclockwise angle is positif. 
	or
	It's the rotation of coordinates system (same point in two coordinates systems).
	Clockwise angle is positif.
	*/
	class CC_2DRotation{
	public:
		CC_2DRotation() { ; }

		//[a, b; c, d] also [cos, -sin; sin, cos]
		void createModel(double angle)
		{
			matrixRot[0] = cos(angle);
			matrixRot[1] = -sin(angle);
			matrixRot[2] = -matrixRot[1];
			matrixRot[3] = matrixRot[0];
		}

		//[a, b; c, d] also [cos, -sin; sin, cos]
		void createModel(double a, double b, double c, double d)
		{
			matrixRot[0] = a;
			matrixRot[1] = b;
			matrixRot[2] = c;
			matrixRot[3] = d;
		}

		//[dst_x, dst_y] = [a, b; c, d] * [src_x, src_y]
		void convert(double src_x, double src_y, double &dst_x, double &dst_y) const
		{
			dst_x = src_x * matrixRot[0] + src_y * matrixRot[1];
			dst_y = src_x * matrixRot[2] + src_y * matrixRot[3];
		}

	private :
		double matrixRot[4];//[a, b; c, d] also [cos, -sin; sin, cos]
	};//end CC_2DRotation

	/*
	It's the simplest model: suppose that the road is plat and in surface Y = 0.
	Convert (u,v) --- of a camera(0, h, 0) with a pitch angle(rx) --- to world coordinate system(X, 0, Z).
	*/
	class CC_SimpleIPM {
	public:
		CC_SimpleIPM() { ; }

		//general supposition h = 1.4m, pitch angle = 0
		void createModel(double fx, double fy, double cu, double cv)
		{
			_fx = fx;
			_fy = fy;
			_cu = cu;
			_cv = cv;
			createPrivateModel(0, 0, 0, 0, 1.4, 0);
		}

		//set h and rx
		void createModel(double fx, double fy, double cu, double cv, double rx, double h)
		{
			_fx = fx;
			_fy = fy;
			_cu = cu;
			_cv = cv;
			_rx = rx;
			_h = h;
			createPrivateModel(rx, 0, 0, 0, h*cos(rx), h*sin(rx));
		}

		//set all parameters
		void createModel(double fx, double fy, double cu, double cv, double rx, double ry, double rz,
			double tx, double ty, double tz)
		{
			_fx = fx;
			_fy = fy;
			_cu = cu;
			_cv = cv;
			createPrivateModel(rx, ry, rz, tx, ty, tz);
		}

		//src_u and src_v are often int. 
		//u,v to x,z
		void convert(double src_u, double src_v, double &dst_x, double &dst_z) const 
		{
			double c2 = M[4] - M[8] * src_v;
			double c3 = (M[11] * src_v - M[7]);
			double c4 = (M[6] - M[10] * src_v);
			double c1 = M[0] - M[8] * src_u;
			double c5 = M[2] - M[10] * src_u;
			double c6 = M[11] * src_u - M[3];
			c5 = c5 / c1;
			c6 = c6 / c1;

			dst_z = (c3 - c6*c2) / (c4 - c5*c2);
			dst_x = (c6 - c5*dst_z);
		}

		//x,z to u,v
		//z(0) * [u;v;1;1/z(0)] = M * [X; 0; Z; 1]
		void convert_inv(double src_x, double src_z, double &dst_u, double &dst_v) const
		{
			dst_u = M[0] * src_x + M[2] * src_z + M[3];
			dst_v = M[4] * src_x + M[6] * src_z + M[7];
			double z = M[8] * src_x + M[10] * src_z + M[11];
			dst_u /= z;
			dst_v /= z;
		}

		void getCameraParam(double &fx, double &fy, double &cu, double &cv)
		{
			fx = _fx, fy = _fy, cu = _cu, cv = _cv;
		}
		void getRxAndH(double &rx, double &h)
		{
			rx = _rx, h = _h;
		}

	private:
		double _fx;//focal length in x direction(col)
		double _fy;//focal length in y direction(row)
		double _cu;//center of x
		double _cv;//center of y
		double _rx;//pitch angle
		double _h;//height of camera
		double M[16];//coodinate of world to (u,v): z(0) * [u;v;1;1/z(0)] = ptr_rCW2UV * [X; 0; Z; 1]

		void createPrivateModel(double rx, double ry, double rz, double tx, double ty, double tz)
		{
			double sx = sin(rx);
			double cx = cos(rx);
			double sy = sin(ry);
			double cy = cos(ry);
			double sz = sin(rz);
			double cz = cos(rz);

			double Rt[16];//rotation and translation
			double A[16];//intrinsic parameters 4 * 4

			//Rz * Rx * Ry
			Rt[0] = +cy*cz - sx*sy*sz;	Rt[1] = -cx*sz;			Rt[2] = +cz*sy + cy*sx*sz;	Rt[3] = tx;
			Rt[4] = cy*sz + cz*sx*sy;	Rt[5] = cx*cz;			Rt[6] = sy*sz - cy*cz*sx;	Rt[7] = ty;
			Rt[8] = -cx*sy;			    Rt[9] = sx;				Rt[10] = +cx*cy;			Rt[11] = tz;
			Rt[12] = 0;					Rt[13] = 0;             Rt[14] = 0;					Rt[15] = 1;
		
			
			for (int i = 0; i < 16; i++)
				A[i] = 0;

			A[0] = _fx, A[2] = _cu;
			A[5] = _fy, A[6] = _cv;
			A[10] = 1;
			A[15] = 1;

			for (int Mi = 0; Mi < 4; Mi++)
			{
				for (int Mj = 0; Mj < 4; Mj++)
				{
					M[Mi * 4 + Mj] = 0;
					for (int k = 0; k < 4; k++)
					{
						M[Mi * 4 + Mj] += A[Mi * 4 + k] * Rt[k * 4 + Mj];
					}
				}
			}
		
		}
	};
	
	/*
	This is from PACPUS. I think it is used for zone France.
	*/
	class CC_lonlat2lamber93 {
	public:
	public:
		CC_lonlat2lamber93() { ; }

		void createModel() {
			GRS_a = 6378137;
			GRS_f = 1 / 298.257222101;
			GRS_b = GRS_a*(1 - GRS_f);
			GRS_bb = GRS_b*GRS_b;
			GRS_aa = 40680631590769.0;
			GRS_e = sqrt((GRS_aa - GRS_bb) / (GRS_aa));
			GLOBAL_n = 0.725607765053267;
			GLOBAL_C = 11754255.4261;
			XS = 700000;
			YS = 12655612.0499;
			PI_180 = CC_PI / 180;
		}

		void convert(double lon, double lat, double & lam93x, double & lam93y) const
		{
			double latiso = atanh(sin(lat)) - GRS_e*atanh(GRS_e*sin(lat));
			double gamma = (lon - 0.0523598775598299)*GLOBAL_n;
			double R = GLOBAL_C * exp(-GLOBAL_n*latiso);

			lam93x = R * sin(gamma);
			lam93y = -R * cos(gamma);
		}

	private:
		double GRS_a;
		double GRS_f;
		double GRS_b;
		double GRS_bb;
		double GRS_aa;
		double GRS_e;
		double GLOBAL_n;
		double GLOBAL_C;
		double XS;
		double YS;
		double PI_180;
	};


	



}

