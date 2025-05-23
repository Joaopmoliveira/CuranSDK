/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2015 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef RBDL_KINEMATICS_H
#define RBDL_KINEMATICS_H

#include "rbdl/rbdl_math.h"
#include <assert.h>
#include <iostream>
#include "rbdl/Logging.h"

namespace RigidBodyDynamics {

/** \page kinematics_page Kinematics
 * All functions related to kinematics are specified in the \ref
 * kinematics_group "Kinematics Module".
 *
 * \note Please note that in the Rigid %Body Dynamics Library all angles
 * are specified in radians.
 *
 * \defgroup kinematics_group Kinematics
 * @{
 *
 * \note Please note that in the Rigid %Body Dynamics Library all angles
 * are specified in radians.
 */

/** \brief Updates and computes velocities and accelerations of the bodies
 *
 * This function updates the kinematic variables such as body velocities
 * and accelerations in the model to reflect the variables passed to this function.
 *
 * \param model the model
 * \param Q     the positional variables of the model
 * \param QDot  the generalized velocities of the joints
 * \param QDDot the generalized accelerations of the joints
 */
RBDL_DLLAPI
void UpdateKinematics (Model &model,
		const Math::VectorNd &Q,
		const Math::VectorNd &QDot,
		const Math::VectorNd &QDDot
		);

/** \brief Selectively updates model internal states of body positions, velocities and/or accelerations.
 *
 * This function updates the kinematic variables such as body velocities and
 * accelerations in the model to reflect the variables passed to this function.
 *
 * In contrast to UpdateKinematics() this function allows to update the model
 * state with values one is interested and thus reduce computations (e.g. only
 * positions, only positions + accelerations, only velocities, etc.).
 
 * \param model the model
 * \param Q     the positional variables of the model
 * \param QDot  the generalized velocities of the joints
 * \param QDDot the generalized accelerations of the joints
 */
RBDL_DLLAPI
void UpdateKinematicsCustom (Model &model,
		const Math::VectorNd *Q,
		const Math::VectorNd *QDot,
		const Math::VectorNd *QDDot
		);

/** \brief Returns the base coordinates of a point given in body coordinates.
 *
 * \param model the rigid body model
 * \param Q the curent genereralized positions
 * \param body_id id of the body for which the point coordinates are expressed
 * \param body_point_position coordinates of the point in body coordinates
 * \param update_kinematics whether UpdateKinematics() should be called
 * or not (default: true)
 *
 * \returns a 3-D vector with coordinates of the point in base coordinates
 */
RBDL_DLLAPI
Math::Vector3d CalcBodyToBaseCoordinates (
		Model &model,
		const Math::VectorNd &Q,
		unsigned int body_id,
		const Math::Vector3d &body_point_position,
		bool update_kinematics = true);

/** \brief Returns the body coordinates of a point given in base coordinates.
 *
 * \param model the rigid body model
 * \param Q the curent genereralized positions
 * \param body_id id of the body for which the point coordinates are expressed
 * \param base_point_position coordinates of the point in base coordinates
 * \param update_kinematics whether UpdateKinematics() should be called or not
 * (default: true).
 *
 * \returns a 3-D vector with coordinates of the point in body coordinates
 */
RBDL_DLLAPI
Math::Vector3d CalcBaseToBodyCoordinates (
		Model &model,
		const Math::VectorNd &Q,
		unsigned int body_id,
		const Math::Vector3d &base_point_position,
		bool update_kinematics = true);

/** \brief Returns the orientation of a given body as 3x3 matrix
 *
 * \param model the rigid body model
 * \param Q the curent genereralized positions
 * \param body_id id of the body for which the point coordinates are expressed
 * \param update_kinematics whether UpdateKinematics() should be called or not
 * (default: true).
 *
 * \returns An orthonormal 3x3 matrix that rotates vectors from base coordinates
 * to body coordinates.
 */
RBDL_DLLAPI
Math::Matrix3d CalcBodyWorldOrientation (
		Model &model,
		const Math::VectorNd &Q,
		const unsigned int body_id,
		bool update_kinematics = true);

/** \brief Computes the point jacobian for a point on a body
 *
 * If a position of a point is computed by a function \f$g(q(t))\f$ for which its
 * time derivative is \f$\frac{d}{dt} g(q(t)) = G(q)\dot{q}\f$ then this
 * function computes the jacobian matrix \f$G(q)\f$.
 *
 * \param model   rigid body model
 * \param Q       state vector of the internal joints
 * \param body_id the id of the body
 * \param point_position the position of the point in body-local data
 * \param G       a matrix of dimensions 3 x \#qdot_size where the result will be stored in
 * \param update_kinematics whether UpdateKinematics() should be called or not (default: true)
 *
 * The result will be returned via the G argument.
 *
 * \note This function only evaluates the entries of G that are non-zero. One
 * Before calling this function one has to ensure that all other values
 * have been set to zero, e.g. by calling G.setZero().
 *
 */ 
RBDL_DLLAPI
void CalcPointJacobian (Model &model,
		const Math::VectorNd &Q,
		unsigned int body_id,
		const Math::Vector3d &point_position,
		Math::MatrixNd &G,
		bool update_kinematics = true
		);

/** \brief Computes the spatial jacobian for a body
 *
 * The spatial velocity of a body at the origin of the base coordinat
 * system can be expressed as \f${}^0 \hat{v}_i = G(q) * \dot{q}\f$. The
 * matrix \f$G(q)\f$ is called the spatial body jacobian of the body and
 * can be computed using this function.
 *
 * \param model   rigid body model
 * \param Q       state vector of the internal joints
 * \param body_id the id of the body
 * \param G       a matrix of size 6 x \#qdot_size where the result will be stored in
 * \param update_kinematics whether UpdateKinematics() should be called or not (default: true)
 *
 * The result will be returned via the G argument and represents the
 * body Jacobian expressed at the origin of the body.
 *
 * \note This function only evaluates the entries of G that are non-zero. One
 * Before calling this function one has to ensure that all other values
 * have been set to zero, e.g. by calling G.setZero().
 */
RBDL_DLLAPI
void CalcBodySpatialJacobian (
		Model &model,
		const Math::VectorNd &Q,
		unsigned int body_id,
		Math::MatrixNd &G,
		bool update_kinematics = true
		);

/** \brief Computes the velocity of a point on a body 
 *
 * \param model   rigid body model
 * \param Q       state vector of the internal joints
 * \param QDot    velocity vector of the internal joints
 * \param body_id the id of the body
 * \param point_position the position of the point in body-local data
 * \param update_kinematics whether UpdateKinematics() should be called or not (default: true)
 *
 * \returns The cartesian velocity of the point in global frame (output)
 */
RBDL_DLLAPI
Math::Vector3d CalcPointVelocity (
		Model &model,
		const Math::VectorNd &Q,
		const Math::VectorNd &QDot,
		unsigned int body_id,
		const Math::Vector3d &point_position,
		bool update_kinematics = true
		);

/** \brief Computes the acceleration of a point on a body 
 *
 * \param model   rigid body model
 * \param Q       state vector of the internal joints
 * \param QDot    velocity vector of the internal joints
 * \param QDDot    velocity vector of the internal joints
 * \param body_id the id of the body
 * \param point_position the position of the point in body-local data
 * \param update_kinematics whether UpdateKinematics() should be called or not (default: true)
 *
 * \returns The cartesian acceleration of the point in global frame (output)
 *
 * The kinematic state of the model has to be updated before valid
 * values can be obtained. This can either be done by calling
 * UpdateKinematics() or setting the last parameter update_kinematics to
 * true (default).
 *
 * \note During the execution of ForwardDynamics() the acceleration
 * is only applied on the root body and propagated form there. Therefore
 * in the internal state the accelerations of the bodies only represent
 * the relative accelerations without any gravitational effects.
 *
 * \warning  If this function is called after ForwardDynamics() without
 * an update of the kinematic state one has to add the gravity
 * acceleration has to be added to the result.
 */

RBDL_DLLAPI
Math::Vector3d CalcPointAcceleration (
		Model &model,
		const Math::VectorNd &Q,
		const Math::VectorNd &QDot,
		const Math::VectorNd &QDDot,
		unsigned int body_id,
		const Math::Vector3d &point_position,
		bool update_kinematics = true
	);

/** \brief Computes the inverse kinematics iteratively using a damped Levenberg-Marquardt method (also known as Damped Least Squares method)
 *
 * \param model rigid body model
 * \param Qinit initial guess for the state
 * \param body_id a vector of all bodies for which we we have kinematic target positions
 * \param body_point a vector of points in body local coordinates that are
 * to be matched to target positions
 * \param target_pos a vector of target positions
 * \param Qres output of the computed inverse kinematics
 * \param step_tol tolerance used for convergence detection
 * \param lambda damping factor for the least squares function
 * \param max_iter maximum number of steps that should be performed
 * \returns true on success, false otherwise
 *
 * This function repeatedly computes
 *   \f[ Qres = Qres + \Delta \theta\f]
 *   \f[ \Delta \theta = G^T (G^T G + \lambda^2 I)^{-1} e \f]
 * where \f$G = G(q) = \frac{d}{dt} g(q(t))\f$ and \f$e\f$ is the
 * correction of the body points so that they coincide with the target
 * positions. The function returns true when \f$||\Delta \theta||_2 \le\f$
 * step_tol or if the error between body points and target gets smaller
 * than step_tol. Otherwise it returns false.
 *
 * The parameter \f$\lambda\f$ is the damping factor that has to
 * be chosen carefully. In case of unreachable positions higher values (e.g
 * 0.9) can be helpful. Otherwise values of 0.0001, 0.001, 0.01, 0.1 might
 * yield good results. See the literature for best practices.
 *
 * \warning The actual accuracy might be rather low (~1.0e-2)! Use this function with a
 * grain of suspicion.
 */
RBDL_DLLAPI
bool InverseKinematics (
		Model &model,
		const Math::VectorNd &Qinit,
		const std::vector<unsigned int>& body_id,
		const std::vector<Math::Vector3d>& body_point,
		const std::vector<Math::Vector3d>& target_pos,
		Math::VectorNd &Qres,
		double step_tol = 1.0e-12,
		double lambda = 0.01,
		unsigned int max_iter = 50
		);

/** @} */

}

/* RBDL_KINEMATICS_H */
#endif 
