#ifndef CONTACT_H
#define CONTACT_H

#include <stdlib.h>
#include <vector>
#include "enum.h"
#include <Eigen/Dense>
#include "constant_values.hpp"

#include <memory>

class Contact {

    public:
        typedef std::shared_ptr<Contact> Ptr;

        Contact(EndEffector _name, Eigen::Affine3d _pose, Eigen::VectorXd _force, Eigen::Vector3d _normal);
        ~Contact();

        EndEffector getEndEffectorName();
        Eigen::Affine3d getPose();
        void setPose(Eigen::VectorXd pos, Eigen::Matrix3d rot);
        Eigen::VectorXd getForce();
        Eigen::Vector3d getNormal();

        void setForce(Eigen::VectorXd _force);

    private:

        EndEffector name;
        Eigen::Affine3d pose;
        Eigen::VectorXd force;
        Eigen::Vector3d normal;
};

#endif
