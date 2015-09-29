#pragma once

#include <opencv2\opencv.hpp>
using namespace cv;

class Segment2d;

class EKalmanFilter{
public:
	/** @brief The constructors.

	@note In C API when CvKalman\* kalmanFilter structure is not needed anymore, it should be released
	with cvReleaseKalman(&kalmanFilter)
	*/
	EKalmanFilter();
	/** @overload
	@param dynamParams Dimensionality of the state.
	@param measureParams Dimensionality of the measurement.
	@param controlParams Dimensionality of the control vector.
	@param type Type of the created matrices that should be CV_32F or CV_64F.
	*/
	EKalmanFilter(int dynamParams, int measureParams, int controlParams = 0, int type = CV_32F);

	/** @brief Re-initializes Kalman filter. The previous content is destroyed.

	@param dynamParams Dimensionality of the state.
	@param measureParams Dimensionality of the measurement.
	@param controlParams Dimensionality of the control vector.
	@param type Type of the created matrices that should be CV_32F or CV_64F.
	*/
	void init(int dynamParams, int measureParams, int controlParams = 0, int type = CV_32F);

	/** @brief Computes a predicted state.

	@param control The optional input control
	*/
	const Mat& predict(const Mat& control = Mat());

	/** @brief Updates the predicted state from the measurement.

	@param measurement The measured system parameters
	*/
	const Mat& correct();

	float residual(float measurement, float hx);

	Mat statePre;           //!< predicted state (x'(k)): x(k)=A*x(k-1)+B*u(k)
	Mat statePost;          //!< corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
	Mat transitionMatrix;   //!< state transition matrix (A)
	Mat controlMatrix;      //!< control matrix (B) (not used if there is no control)
	Mat measurementMatrix;  //!< measurement matrix (H)
	Mat processNoiseCov;    //!< process noise covariance matrix (Q)
	Mat measurementNoiseCov;//!< measurement noise covariance matrix (R)
	Mat errorCovPre;        //!< priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q)*/
	Mat gain;               //!< Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)
	Mat errorCovPost;       //!< posteriori error estimate covariance matrix (P(k)): P(k)=(I-K(k)*H)*P'(k)

	// temporary matrices
	Mat temp1;
	Mat temp2;
	Mat temp3;
	Mat temp4;
	float temp5;

};