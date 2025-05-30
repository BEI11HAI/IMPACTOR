#pragma once
#include <Eigen/Geometry>
#include <iostream>
namespace so3_controller {

class SO3Controller {
 private:
  // input
  double          mass_;
  double          mass_l_; //  load mass
  double          l_length_; //load length 
  double          g_;
  // Eigen::Vector3d pos_;
  // Eigen::Vector3d vel_;
  // Eigen::Vector3d acc_;
  Eigen::Vector3d load_pos_;
  Eigen::Vector3d load_vel_;
  Eigen::Vector3d load_acc_;
  Eigen::Vector3d load_q_;
  Eigen::Vector3d load_dq_;
  // output
  Eigen::Vector3d    force_;
  Eigen::Quaterniond quat_;
 public:
  SO3Controller(const double& mass,const double& mass_l, const double& l_length, const double& g) : 
    mass_(mass), mass_l_(mass_l),l_length_(l_length),g_(g) {};
  ~SO3Controller() {};
  // update state from sensors
  // inline void setPos(const Eigen::Vector3d& pos) {
  //   pos_ = pos;
  // }
  // inline void setVel(const Eigen::Vector3d& vel) {
  //   vel_ = vel;
  // }
  // inline void setAcc(const Eigen::Vector3d& acc) {
  //   acc_ = acc;
  // }
  inline void setLoadPos(const Eigen::Vector3d& pos) {
    load_pos_ = pos;
  }
  inline void setLoadVel(const Eigen::Vector3d& vel) {
    load_vel_ = vel;
  }
  inline void setLoadAcc(const Eigen::Vector3d& acc) {
    load_acc_ = acc;
  }
  inline void setLoadq(const Eigen::Vector3d& q) {
    load_q_ = q;
  }
  inline void setLoaddq(const Eigen::Vector3d& dq) {
    load_dq_ = dq;
  }

  inline Eigen::Vector3d getF() const {
    return force_;
  }
  inline Eigen::Quaterniond getQ() const {
    return quat_;
  }

  // pid
  inline void calculateLoadControl(const Eigen::Vector3d& des_load_pos,
                               const Eigen::Vector3d& des_load_vel,
                               const Eigen::Vector3d& des_load_acc,
                               const double& des_yaw, const double& des_yaw_dot,
                               const Eigen::Vector3d& kx,
                               const Eigen::Vector3d& kv,
                               const Eigen::Vector3d& kq,const Eigen::Vector3d& kw) {
    // Eigen::Vector3d totalError = (des_load_pos - load_pos_) + (des_load_vel - load_vel_) + (des_load_acc - load_acc_);
    // Eigen::Vector3d ka(fabs(totalError[0]) > 3 ? 0 : (fabs(totalError[0]) * 0.2),
    //                    fabs(totalError[1]) > 3 ? 0 : (fabs(totalError[1]) * 0.2),
    //                    fabs(totalError[2]) > 3 ? 0 : (fabs(totalError[2]) * 0.2));

    Eigen::Vector3d error =
      kx.asDiagonal() * (des_load_pos - load_pos_) + kv.asDiagonal() * (des_load_vel - load_vel_);
      // + (mass_+mass_l_)* ka.asDiagonal() * (des_load_acc - load_acc_);

    Eigen::Vector3d A =  error + (mass_ + mass_l_)*(des_load_acc + g_ * Eigen::Vector3d(0, 0, 1)) 
    + mass_ * l_length_*load_dq_.dot(load_dq_)*load_q_;

    Eigen::Vector3d fn_ = A.dot(load_q_)*load_q_;
    Eigen::Vector3d q_d = -A/A.norm();

    Eigen::Matrix3d q_vee(Eigen::Matrix3d::Zero());

    q_vee(2, 1) = load_q_(0);
    q_vee(1, 2) = -load_q_(0);
    q_vee(0, 2) = load_q_(1);
    q_vee(2, 0) = -load_q_(1);
    q_vee(1, 0) = load_q_(2);
    q_vee(0, 1) = -load_q_(2);
    
    Eigen::Vector3d fpd = kq.asDiagonal()*(q_vee*q_vee*q_d) + kw.asDiagonal()*load_dq_;
    force_ = fn_ + fpd;
    // Limit control angle to 45 degree
    double          theta = M_PI / 2;
    double          c     = cos(theta);
    Eigen::Vector3d f;
    // f.noalias() = kx.asDiagonal() * (des_load_pos - des_load_vel) +
    //               kv.asDiagonal() * (des_load_vel - load_vel_) +
    //                (mass_+mass_l_) * des_load_acc +
    //                (mass_+mass_l_) * ka.asDiagonal() * (des_load_acc - load_acc_);
    f = force_;
    // if (Eigen::Vector3d(0, 0, 1).dot(force_ / force_.norm()) < c) {
      
    //   double nf        = f.norm();
    //   double A         = c * c * nf * nf - f(2) * f(2);
    //   double B         = 2 * (c * c - 1) * f(2) *  (mass_+mass_l_) * g_;
    //   double C         = (c * c - 1) *  (mass_+mass_l_) *  (mass_+mass_l_) * g_ * g_;
    //   double s         = (-B + sqrt(B * B - 4 * A * C)) / (2 * A);
    //   force_.noalias() = s * f +  (mass_+mass_l_) * g_ * Eigen::Vector3d(0, 0, 1);
    // }
    // Limit control angle to 45 degree

    Eigen::Vector3d b1c, b2c, b3c;
    Eigen::Vector3d b1d(cos(des_yaw), sin(des_yaw), 0);

    if (force_.norm() > 1e-6)
      b3c.noalias() = force_.normalized();
    else
      b3c.noalias() = Eigen::Vector3d(0, 0, 1);

    b2c.noalias() = b3c.cross(b1d).normalized();
    b1c.noalias() = b2c.cross(b3c).normalized();

    Eigen::Matrix3d R;
    R << b1c, b2c, b3c;

    quat_ = Eigen::Quaterniond(R);
  }

  /*No payload PID Contoller
  inline void calculateControl(const Eigen::Vector3d& des_pos,
                               const Eigen::Vector3d& des_vel,
                               const Eigen::Vector3d& des_acc,
                               const double& des_yaw, const double& des_yaw_dot,
                               const Eigen::Vector3d& kx,
                               const Eigen::Vector3d& kv) {
    Eigen::Vector3d totalError = (des_pos - pos_) + (des_vel - vel_) + (des_acc - acc_);
    Eigen::Vector3d ka(fabs(totalError[0]) > 3 ? 0 : (fabs(totalError[0]) * 0.2),
                       fabs(totalError[1]) > 3 ? 0 : (fabs(totalError[1]) * 0.2),
                       fabs(totalError[2]) > 3 ? 0 : (fabs(totalError[2]) * 0.2));
    force_.noalias() =
      kx.asDiagonal() * (des_pos - pos_) + kv.asDiagonal() * (des_vel - vel_) +
      mass_ * 
      //(Eigen::Vector3d(1, 1, 1) - ka).asDiagonal() *
      (des_acc) +
      mass_ * ka.asDiagonal() * (des_acc - acc_) +
      mass_ * g_ * Eigen::Vector3d(0, 0, 1);
    // Limit control angle to 45 degree
    double          theta = M_PI / 2;
    double          c     = cos(theta);
    Eigen::Vector3d f;
    f.noalias() = kx.asDiagonal() * (des_pos - pos_) +
                  kv.asDiagonal() * (des_vel - vel_) +
                  mass_ * des_acc +
                  mass_ * ka.asDiagonal() * (des_acc - acc_);
    if (Eigen::Vector3d(0, 0, 1).dot(force_ / force_.norm()) < c) {
      double nf        = f.norm();
      double A         = c * c * nf * nf - f(2) * f(2);
      double B         = 2 * (c * c - 1) * f(2) * mass_ * g_;
      double C         = (c * c - 1) * mass_ * mass_ * g_ * g_;
      double s         = (-B + sqrt(B * B - 4 * A * C)) / (2 * A);
      force_.noalias() = s * f + mass_ * g_ * Eigen::Vector3d(0, 0, 1);
    }
    // Limit control angle to 45 degree

    Eigen::Vector3d b1c, b2c, b3c;
    Eigen::Vector3d b1d(cos(des_yaw), sin(des_yaw), 0);

    if (force_.norm() > 1e-6)
      b3c.noalias() = force_.normalized();
    else
      b3c.noalias() = Eigen::Vector3d(0, 0, 1);

    b2c.noalias() = b3c.cross(b1d).normalized();
    b1c.noalias() = b2c.cross(b3c).normalized();

    Eigen::Matrix3d R;
    R << b1c, b2c, b3c;

    quat_ = Eigen::Quaterniond(R);
  }*/
};

} // namespace so3_controller
