#ifndef CONTACT_H
#define CONTACT_H

#include <stdlib.h>
#include <vector>
#include "enum.h"
#include <Eigen/Dense>
#include "constant_values.hpp"

class Contact {

    public:

        Contact(EndEffector _name, Eigen::Affine3d _pose, Eigen::Vector3d _force, Eigen::Vector3d _normal);
        ~Contact();

        EndEffector getEndEffectorName();
        Eigen::Affine3d getPose();
        void setPose(Eigen::VectorXd pos, Eigen::Matrix3d rot);
        Eigen::Vector3d getForce();
        Eigen::Vector3d getNormal();

        void setForce(Eigen::Vector3d _force);

    private:

        EndEffector name;
        Eigen::Affine3d pose;
        Eigen::Vector3d force;
        Eigen::Vector3d normal;
};

#endif
