#ifndef RIGTFORM_H
#define RIGTFORM_H

#include <iostream>
#include <cassert>

#include "matrix4.h"
#include "quat.h"

class RigTForm {
  Cvec3 t_; // translation component
  Quat r_;  // rotation component represented as a quaternion

public:
  RigTForm() : t_(0) {
    assert(norm2(Quat(1,0,0,0) - r_) < CS175_EPS2);
  }

  RigTForm(const Cvec3& t, const Quat& r) {
    //TODO
	  this->t_ = t;
	  this->r_ = r;
  }

  explicit RigTForm(const Cvec3& t) {
    // TODO
	  this->r_ = Quat();
	  this->t_ = t;
  }

  explicit RigTForm(const Quat& r) {
	  // TODO
	  this->r_ = r;
	  this->t_ = Cvec3();
  }

  Cvec3 getTranslation() const {
    return t_;
  }

  Quat getRotation() const {
    return r_;
  }

  RigTForm& setTranslation(const Cvec3& t) {
    t_ = t;
    return *this;
  }

  RigTForm& setRotation(const Quat& r) {
    r_ = r;
    return *this;
  }

  Cvec4 operator * (const Cvec4& a) const {
	  // TODO
	  Cvec4 r(0);
	  r = this->r_ * a + Cvec4(this->t_, 0);
	  return r;
  }

  RigTForm operator * (const RigTForm& a) const {
	  // TODO
	  Quat new_r;
	  Cvec3 new_t;
	  new_t = this->t_ + this->r_ * a.t_;
	  new_r = this->r_* a.r_;
	  RigTForm new_RigT = RigTForm(new_t, new_r);
	  return new_RigT;
  }
};

inline RigTForm inv(const RigTForm& tform) {
  // TODO
	Quat new_r;
	Cvec3 new_t;
	new_t = (inv(tform.getRotation()) *tform.getTranslation()) *(-1.0);
	new_r = inv(tform.getRotation());
	RigTForm new_RigT = RigTForm(new_t, new_r);
	return new_RigT;
}

inline RigTForm transFact(const RigTForm& tform) {
  return RigTForm(tform.getTranslation());
}

inline RigTForm linFact(const RigTForm& tform) {
  return RigTForm(tform.getRotation());
}

inline Matrix4 rigTFormToMatrix(const RigTForm& tform) {
	// TODO
	Matrix4 T = Matrix4::makeTranslation(tform.getTranslation());
	Matrix4 R = quatToMatrix(tform.getRotation());
	return T*R;
}

//Do M to O with respent to A 
inline RigTForm doMtoOwrtA(const RigTForm &m, const RigTForm &objRbt, const RigTForm &affine) {
	return affine * m * inv(affine)*objRbt;
}

//return O_T * E_R, make affine matrix
inline RigTForm makeMixedFrame(const RigTForm &objRbt, const RigTForm &eyeRbt) {
	//return RigTForm(objRbt.getTranslation(), eyeRbt.getRotation());
	return transFact(objRbt) * linFact(eyeRbt);
}
#endif
