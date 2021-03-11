#ifndef STANCE_H
#define STANCE_H

#include <stdlib.h>
#include <iostream>
#include <vector>
#include "enum.h"
#include <Eigen/Dense>
#include "Contact.hpp"
#include "constant_values.hpp"

#include <memory>

class Stance {

    private:
        std::vector<std::shared_ptr<Contact>> contactsList;

    public:

        Stance();

        std::shared_ptr<Contact> getContact(int i);
        std::vector<std::shared_ptr<Contact>> getContacts();
        void addContact(std::shared_ptr<Contact> c);
        int getSize();
        void clear();

        std::vector<EndEffector> retrieveActiveEndEffectors();
        bool isActiveEndEffector(EndEffector pk);
        Eigen::Affine3d retrieveContactPose(EndEffector pk);
        Eigen::Vector3d retrieveContactNormal(EndEffector pk);

        void print();

};

#endif
