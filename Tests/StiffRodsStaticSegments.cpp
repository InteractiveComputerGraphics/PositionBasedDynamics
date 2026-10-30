#include "PositionBasedDynamics/PositionBasedElasticRods.h"

#include <array>
#include <cmath>
#include <iostream>

namespace
{
using Solver = PBD::DirectPositionBasedSolverForStiffRods;

struct Segment : PBD::RodSegment
{
    Real mass;
    Vector3r position;
    Vector3r inertia = Vector3r::Constant(Real(0.1));
    Quaternionr rotation = Quaternionr::Identity();

    Segment(Real m, Real y) : mass(m), position(0, y, 0) {}
    bool isDynamic() override { return mass != Real(0); }
    Real Mass() override { return mass; }
    const Vector3r &InertiaTensor() override { return inertia; }
    const Vector3r &Position() override { return position; }
    const Quaternionr &Rotation() override { return rotation; }
};

struct Constraint : PBD::RodConstraint
{
    unsigned int indices[2];
    Eigen::Matrix<Real, 3, 4, Eigen::DontAlign> info;
    Vector3r restDarboux, stiffness, stretchCompliance, bendingCompliance;

    Constraint(unsigned int a, unsigned int b) : indices{a, b} {}
    unsigned int segmentIndex(unsigned int i) override { return indices[i]; }
    Eigen::Matrix<Real, 3, 4, Eigen::DontAlign> &getConstraintInfo() override { return info; }
    Real getAverageSegmentLength() override { return Real(1); }
    Vector3r &getRestDarbouxVector() override { return restDarboux; }
    Vector3r &getStiffnessCoefficientK() override { return stiffness; }
    Vector3r &getStretchCompliance() override { return stretchCompliance; }
    Vector3r &getBendingAndTorsionCompliance() override { return bendingCompliance; }
};

struct Rod
{
    std::array<Segment, 3> segments;
    std::array<Constraint, 2> constraints;
    std::vector<PBD::RodSegment *> segmentPointers;
    std::vector<PBD::RodConstraint *> constraintPointers;
    PBD::Interval *intervals = nullptr;
    int intervalCount = 0;
    std::list<PBD::Node *> *forward = nullptr, *backward = nullptr;
    PBD::Node *root = nullptr;
    std::vector<Vector6r> rhs, lambda;
    std::vector<std::vector<Matrix3r>> jacobians;
    std::vector<Vector3r> positionCorrections;
    std::vector<Quaternionr> rotationCorrections;

    Rod(Real firstMass, Real lastMass)
        : segments{{Segment(firstMass, 0), Segment(1, 1), Segment(lastMass, 2)}},
          constraints{{Constraint(0, 1), Constraint(1, 2)}},
          segmentPointers{&segments[0], &segments[1], &segments[2]},
          constraintPointers{&constraints[0], &constraints[1]}
    {
        Solver::init_DirectPositionBasedSolverForStiffRodsConstraint(
            constraintPointers, segmentPointers, intervals, intervalCount,
            forward, backward, root,
            {Vector3r(0, Real(0.5), 0), Vector3r(0, Real(1.5), 0)},
            {Real(0.1), Real(0.1)}, {Real(1e8), Real(1e8)}, {Real(5e7), Real(5e7)},
            rhs, lambda, jacobians, positionCorrections, rotationCorrections);
    }

    ~Rod()
    {
        for (int i = 0; i < intervalCount; ++i)
            for (PBD::Node *node : forward[i])
                if (node->parent != nullptr)
                    delete node;
        delete[] intervals;
        delete[] forward;
        delete[] backward;
        delete[] root;
    }

    void solve()
    {
        Solver::initBeforeProjection_DirectPositionBasedSolverForStiffRodsConstraint(
            constraintPointers, Real(1000), lambda);
        Solver::update_DirectPositionBasedSolverForStiffRodsConstraint(
            constraintPointers, segmentPointers);
        Solver::solve_DirectPositionBasedSolverForStiffRodsConstraint(
            constraintPointers, segmentPointers, intervals, intervalCount,
            forward, backward, rhs, lambda, jacobians, positionCorrections, rotationCorrections);
    }
};

bool checkCorrection(const char *name, Real firstMass, Real lastMass, bool twist)
{
    Rod rod(firstMass, lastMass);
    if (twist)
        rod.segments[1].rotation = Quaternionr(AngleAxisr(Real(0.2), Vector3r::UnitY()));
    else
        rod.segments[1].position.x() = Real(0.1);

    const auto initialPositionCorrections = rod.positionCorrections;
    const auto initialRotationCorrections = rod.rotationCorrections;
    rod.solve();

    const Vector3r &dx = rod.positionCorrections[1];
    const Quaternionr &dq = rod.rotationCorrections[1];
    bool passed = dx.allFinite() && dq.coeffs().allFinite();
    if (twist)
    {
        Quaternionr corrected = rod.segments[1].rotation;
        corrected.coeffs() += dq.coeffs();
        corrected.normalize();
        // An unwritten quaternion correction is Identity, so its norm alone
        // would not detect a skipped dynamic segment.
        passed = passed && dq.y() < Real(-1e-5)
            && std::abs(corrected.y()) < std::abs(rod.segments[1].rotation.y());
    }
    else
        passed = passed && dx.x() < Real(-1e-4)
            && std::abs(rod.segments[1].position.x() + dx.x()) < Real(0.1);

    for (size_t i = 0; i < rod.segments.size(); ++i)
    {
        if (!rod.segments[i].isDynamic())
        {
            passed = passed
                && rod.positionCorrections[i] == initialPositionCorrections[i]
                && rod.rotationCorrections[i].coeffs() == initialRotationCorrections[i].coeffs();
        }
    }
    std::cout << (passed ? "PASS " : "FAIL ") << name
              << (twist ? " twist" : " translation")
              << ": dx=" << dx.transpose() << ", dq=" << dq.coeffs().transpose() << '\n';
    return passed;
}
}

int main()
{
    bool passed = true;
    for (bool twist : {false, true})
    {
        passed = checkCorrection("all dynamic", 1, 1, twist) && passed;
        passed = checkCorrection("one fixed end", 0, 1, twist) && passed;
        passed = checkCorrection("two fixed ends", 0, 0, twist) && passed;
    }
    return passed ? 0 : 1;
}
