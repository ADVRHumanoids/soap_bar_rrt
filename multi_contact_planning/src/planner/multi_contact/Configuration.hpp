#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <stdlib.h>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>

typedef Eigen::Matrix<double, 6, 1> Vector6d;

class Configuration {

	private:

		Vector6d q_fb;
		Eigen::VectorXd q_jnt;

	public:

		Configuration();
		Configuration(int dof);
        ~Configuration();

        Configuration& operator= (const Configuration& c);

    	Eigen::Vector3d getFBPosition();
		Eigen::Vector3d getFBOrientation();
		void setFBPosition(Eigen::Vector3d p);
		void setFBOrientation(Eigen::Vector3d o);
		double getJointComponent(int i);		
		void setJointComponent(int i, double val);
		Eigen::VectorXd getJointValues();
		void setJointValues(Eigen::VectorXd _q_jnt);
		
};

#endif
