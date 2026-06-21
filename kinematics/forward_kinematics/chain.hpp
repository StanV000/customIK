#pragma once
#include <vector>
#include "joint.hpp"
#include "../../math/transform.hpp"

class Chain {
    public:
        Transform base;
        std::vector<Joint> joints;

        Chain(const Transform &base, const std::vector<Joint> &joints)
            : base(base), joints(joints) {} 

        void addjoint(const Joint &joint) {
            joints.push_back(joint);
        }

        void setAngle(int jointIndex, double angle) {
            if (jointIndex >= 0 && jointIndex < joints.size()) {
                joints[jointIndex].setAngle(angle);
            }
        }


        Transform forwardKinematics() const {
            Transform T = base;
            for (const auto &joint : joints) {
                T = T * joint.localTransform();
            }
            return T;
        }

};