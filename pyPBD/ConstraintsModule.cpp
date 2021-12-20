#include "common.h"

#include <Simulation/Constraints.h>

#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/stl_bind.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>

namespace py = pybind11;

#define CONSTRAINT(name, base) \
    py::class_<PBD::name, PBD::base>(m_sub, #name) \
        .def(py::init<>()) \
        .def_readwrite_static("TYPE_ID", &PBD::name::TYPE_ID) 

#define CONSTRAINT_JOINTINFO(name, base) \
    CONSTRAINT(name, base) \
        .def_readwrite("jointInfo", &PBD::name::m_jointInfo)

void ConstraintsModule(py::module m_sub) 
{

    py::class_<PBD::Constraint>(m_sub, "Constraint")
        .def_readwrite("bodies", &PBD::Constraint::m_bodies)
        .def("getTypeId", &PBD::Constraint::getTypeId)
        .def("initConstraintBeforeProjection", &PBD::Constraint::initConstraintBeforeProjection)
        .def("updateConstraint", &PBD::Constraint::updateConstraint)
        .def("solvePositionConstraint", &PBD::Constraint::solvePositionConstraint)
        .def("solveVelocityConstraint", &PBD::Constraint::solveVelocityConstraint);

    CONSTRAINT_JOINTINFO(BallJoint, Constraint);
    CONSTRAINT_JOINTINFO(BallOnLineJoint, Constraint);
    CONSTRAINT_JOINTINFO(HingeJoint, Constraint);
    CONSTRAINT_JOINTINFO(UniversalJoint, Constraint);
    CONSTRAINT_JOINTINFO(SliderJoint, Constraint);

    py::class_<PBD::MotorJoint, PBD::Constraint>(m_sub, "MotorJoint")
        .def("getTarget", &PBD::MotorJoint::getTarget)
        .def("setTarget", &PBD::MotorJoint::setTarget)
        .def("getTargetSequence", &PBD::MotorJoint::getTargetSequence)
        .def("setTargetSequence", &PBD::MotorJoint::setTargetSequence)
        .def("getRepeatSequence", &PBD::MotorJoint::getRepeatSequence)
        .def("setRepeatSequence", &PBD::MotorJoint::setRepeatSequence);

    CONSTRAINT_JOINTINFO(TargetPositionMotorSliderJoint, MotorJoint);
    CONSTRAINT_JOINTINFO(TargetVelocityMotorSliderJoint, MotorJoint);
    CONSTRAINT_JOINTINFO(TargetAngleMotorHingeJoint, MotorJoint);
    CONSTRAINT_JOINTINFO(TargetVelocityMotorHingeJoint, MotorJoint);
    CONSTRAINT_JOINTINFO(DamperJoint, Constraint)
        .def_readwrite("stiffness", &PBD::DamperJoint::m_stiffness)
        .def_readwrite("lambda", &PBD::DamperJoint::m_lambda);
    CONSTRAINT_JOINTINFO(RigidBodyParticleBallJoint, Constraint);
    CONSTRAINT_JOINTINFO(RigidBodySpring, Constraint)
        .def_readwrite("restLength", &PBD::RigidBodySpring::m_restLength)
        .def_readwrite("stiffness", &PBD::RigidBodySpring::m_stiffness)
        .def_readwrite("lambda", &PBD::RigidBodySpring::m_lambda);
    CONSTRAINT_JOINTINFO(DistanceJoint, Constraint)
        .def_readwrite("restLength", &PBD::DistanceJoint::m_restLength);

    CONSTRAINT(DistanceConstraint, Constraint)
        .def_readwrite("stiffness", &PBD::DistanceConstraint::m_stiffness)
        .def_readwrite("restLength", &PBD::DistanceConstraint::m_restLength);
    CONSTRAINT(DistanceConstraint_XPBD, Constraint)
        .def_readwrite("stiffness", &PBD::DistanceConstraint_XPBD::m_stiffness)
        .def_readwrite("restLength", &PBD::DistanceConstraint_XPBD::m_restLength)
        .def_readwrite("lambda", &PBD::DistanceConstraint_XPBD::m_lambda);
    CONSTRAINT(DihedralConstraint, Constraint)
        .def_readwrite("stiffness", &PBD::DihedralConstraint::m_stiffness)
        .def_readwrite("restAngle", &PBD::DihedralConstraint::m_restAngle);
    CONSTRAINT(IsometricBendingConstraint, Constraint)
        .def_readwrite("stiffness", &PBD::IsometricBendingConstraint::m_stiffness)
        .def_readwrite("Q", &PBD::IsometricBendingConstraint::m_Q);
    CONSTRAINT(IsometricBendingConstraint_XPBD, Constraint)
        .def_readwrite("stiffness", &PBD::IsometricBendingConstraint_XPBD::m_stiffness)
        .def_readwrite("Q", &PBD::IsometricBendingConstraint_XPBD::m_Q)
        .def_readwrite("lambda", &PBD::IsometricBendingConstraint_XPBD::m_lambda);
    CONSTRAINT(FEMTriangleConstraint, Constraint)
        .def_readwrite("xxStiffness", &PBD::FEMTriangleConstraint::m_xxStiffness)
        .def_readwrite("yyStiffness", &PBD::FEMTriangleConstraint::m_yyStiffness)
        .def_readwrite("xyStiffness", &PBD::FEMTriangleConstraint::m_xyStiffness)
        .def_readwrite("xyPoissonRatio", &PBD::FEMTriangleConstraint::m_xyPoissonRatio)
        .def_readwrite("yxPoissonRatio", &PBD::FEMTriangleConstraint::m_yxPoissonRatio)
        .def_readwrite("area", &PBD::FEMTriangleConstraint::m_area)
        .def_readwrite("invRestMat", &PBD::FEMTriangleConstraint::m_invRestMat);
    CONSTRAINT(StrainTriangleConstraint, Constraint)
        .def_readwrite("xxStiffness", &PBD::StrainTriangleConstraint::m_xxStiffness)
        .def_readwrite("yyStiffness", &PBD::StrainTriangleConstraint::m_yyStiffness)
        .def_readwrite("xyStiffness", &PBD::StrainTriangleConstraint::m_xyStiffness)
        .def_readwrite("normalizeStretch", &PBD::StrainTriangleConstraint::m_normalizeStretch)
        .def_readwrite("normalizeShear", &PBD::StrainTriangleConstraint::m_normalizeShear)
        .def_readwrite("invRestMat", &PBD::StrainTriangleConstraint::m_invRestMat);
    CONSTRAINT(VolumeConstraint, Constraint)
        .def_readwrite("stiffness", &PBD::VolumeConstraint::m_stiffness)
        .def_readwrite("restVolume", &PBD::VolumeConstraint::m_restVolume);
    CONSTRAINT(VolumeConstraint_XPBD, Constraint)
        .def_readwrite("stiffness", &PBD::VolumeConstraint_XPBD::m_stiffness)
        .def_readwrite("restVolume", &PBD::VolumeConstraint_XPBD::m_restVolume)
        .def_readwrite("lambda", &PBD::VolumeConstraint_XPBD::m_lambda);
    CONSTRAINT(FEMTetConstraint, Constraint)
        .def_readwrite("stiffness", &PBD::FEMTetConstraint::m_stiffness)
        .def_readwrite("poissonRatio", &PBD::FEMTetConstraint::m_poissonRatio)
        .def_readwrite("volume", &PBD::FEMTetConstraint::m_volume)
        .def_readwrite("invRestMat", &PBD::FEMTetConstraint::m_invRestMat);
    CONSTRAINT(StrainTetConstraint, Constraint)
        .def_readwrite("stretchStiffness", &PBD::StrainTetConstraint::m_stretchStiffness)
        .def_readwrite("shearStiffness", &PBD::StrainTetConstraint::m_shearStiffness)
        .def_readwrite("normalizeStretch", &PBD::StrainTetConstraint::m_normalizeStretch)
        .def_readwrite("normalizeShear", &PBD::StrainTetConstraint::m_normalizeShear)
        .def_readwrite("invRestMat", &PBD::StrainTetConstraint::m_invRestMat);
    py::class_<PBD::ShapeMatchingConstraint, PBD::Constraint>(m_sub, "ShapeMatchingConstraint")
        .def(py::init<const unsigned int>()) 
        .def_readwrite("stiffness", &PBD::ShapeMatchingConstraint::m_stiffness)
        .def_readwrite_static("TYPE_ID", &PBD::ShapeMatchingConstraint::TYPE_ID)
        .def_readwrite("restCm", &PBD::ShapeMatchingConstraint::m_restCm);
    CONSTRAINT(StretchShearConstraint, Constraint)
        .def_readwrite("stretchingStiffness", &PBD::StretchShearConstraint::m_stretchingStiffness)
        .def_readwrite("shearingStiffness1", &PBD::StretchShearConstraint::m_shearingStiffness1)
        .def_readwrite("shearingStiffness2", &PBD::StretchShearConstraint::m_shearingStiffness2)
        .def_readwrite("restLength", &PBD::StretchShearConstraint::m_restLength);
    CONSTRAINT(BendTwistConstraint, Constraint)
        .def_readwrite("twistingStiffness", &PBD::BendTwistConstraint::m_twistingStiffness)
        .def_readwrite("bendingStiffness1", &PBD::BendTwistConstraint::m_bendingStiffness1)
        .def_readwrite("bendingStiffness2", &PBD::BendTwistConstraint::m_bendingStiffness2)
        .def_readwrite("restDarbouxVector", &PBD::BendTwistConstraint::m_restDarbouxVector);
    CONSTRAINT(StretchBendingTwistingConstraint, Constraint)
        .def_readwrite("averageRadius", &PBD::StretchBendingTwistingConstraint::m_averageRadius)
        .def_readwrite("averageSegmentLength", &PBD::StretchBendingTwistingConstraint::m_averageSegmentLength)
        .def_readwrite("restDarbouxVector", &PBD::StretchBendingTwistingConstraint::m_restDarbouxVector)
        .def_readwrite("stiffnessCoefficientK", &PBD::StretchBendingTwistingConstraint::m_stiffnessCoefficientK)
        .def_readwrite("stretchCompliance", &PBD::StretchBendingTwistingConstraint::m_stretchCompliance)
        .def_readwrite("bendingAndTorsionCompliance", &PBD::StretchBendingTwistingConstraint::m_bendingAndTorsionCompliance)
        .def_readwrite("lambdaSum", &PBD::StretchBendingTwistingConstraint::m_lambdaSum);
    CONSTRAINT(DirectPositionBasedSolverForStiffRodsConstraint, Constraint)
        .def("initConstraint", &PBD::DirectPositionBasedSolverForStiffRodsConstraint::initConstraint);

    py::class_<PBD::RigidBodyContactConstraint>(m_sub, "RigidBodyContactConstraint")
        .def_readwrite("bodies", &PBD::RigidBodyContactConstraint::m_bodies)
        .def_readwrite("stiffness", &PBD::RigidBodyContactConstraint::m_stiffness)
        .def_readwrite("frictionCoeff", &PBD::RigidBodyContactConstraint::m_frictionCoeff)
        .def_readwrite("sum_impulses", &PBD::RigidBodyContactConstraint::m_sum_impulses)
        .def_readwrite("constraintInfo", &PBD::RigidBodyContactConstraint::m_constraintInfo)
        .def("getTypeId", &PBD::RigidBodyContactConstraint::getTypeId)
        .def("initConstraint", &PBD::RigidBodyContactConstraint::initConstraint)
        .def("solveVelocityConstraint", &PBD::RigidBodyContactConstraint::solveVelocityConstraint);

    py::class_<PBD::ParticleRigidBodyContactConstraint>(m_sub, "ParticleRigidBodyContactConstraint")
        .def_readwrite("bodies", &PBD::ParticleRigidBodyContactConstraint::m_bodies)
        .def_readwrite("stiffness", &PBD::ParticleRigidBodyContactConstraint::m_stiffness)
        .def_readwrite("frictionCoeff", &PBD::ParticleRigidBodyContactConstraint::m_frictionCoeff)
        .def_readwrite("sum_impulses", &PBD::ParticleRigidBodyContactConstraint::m_sum_impulses)
        .def_readwrite("constraintInfo", &PBD::ParticleRigidBodyContactConstraint::m_constraintInfo)
        .def("initConstraint", &PBD::ParticleRigidBodyContactConstraint::initConstraint)
        .def("solveVelocityConstraint", &PBD::ParticleRigidBodyContactConstraint::solveVelocityConstraint);

    py::class_<PBD::ParticleTetContactConstraint>(m_sub, "ParticleTetContactConstraint")
        .def_readwrite("bodies", &PBD::ParticleTetContactConstraint::m_bodies)
        .def_readwrite("solidIndex", &PBD::ParticleTetContactConstraint::m_solidIndex)
        .def_readwrite("tetIndex", &PBD::ParticleTetContactConstraint::m_tetIndex)
        .def_readwrite("frictionCoeff", &PBD::ParticleTetContactConstraint::m_frictionCoeff)
        .def_readwrite("bary", &PBD::ParticleTetContactConstraint::m_bary)
        .def_readwrite("lambda", &PBD::ParticleTetContactConstraint::m_lambda)
        .def_readwrite("constraintInfo", &PBD::ParticleTetContactConstraint::m_constraintInfo)
        .def_readwrite("x", &PBD::ParticleTetContactConstraint::m_x)
        .def_readwrite("v", &PBD::ParticleTetContactConstraint::m_v)
        .def("initConstraint", &PBD::ParticleTetContactConstraint::initConstraint)
        .def("solvePositionConstraint", &PBD::ParticleTetContactConstraint::solvePositionConstraint)
        .def("solveVelocityConstraint", &PBD::ParticleTetContactConstraint::solveVelocityConstraint);
}