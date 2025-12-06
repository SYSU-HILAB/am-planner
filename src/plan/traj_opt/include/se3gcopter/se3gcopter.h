/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to
   deal in the Software without restriction, including without limitation the
   rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
   sell copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
   IN THE SOFTWARE.
*/

#ifndef SE3GCOPTER_H
#define SE3GCOPTER_H

#include "se3gcopter/minco_arm.h"
#include "se3gcopter/minco_base.h"

// @article{WANG2021GCOPTER,
//     title={Geometrically Constrained Trajectory Optimization for
//     Multicopters}, author={Wang, Zhepei and Zhou, Xin and Xu, Chao and Gao,
//     Fei}, journal={arXiv preprint arXiv:2103.00190}, year={2021}
// }

bool debug = true;

class SE3GCOPTER
{
 private:
  // Use C2 or Cinf diffeo
  bool c2dfm_;

  // Use soft time or not
  bool softT_;

  // Weight for time regularization term
  double rho_;

  // Fixed total time
  double sumT_;

  // constrain mode
  Mode mode_;

  // Minimum Jerk Optimizer
  MINCO_S3_BASE jerkOpt_;
  // Minimum Arm Jerk Optimizer
  MINCO_S3_ARM armOpt_;

  static WorkspaceProbabilityModel workspaceModel_;

  // Temp variables for problem solving
  Eigen::MatrixXd iState_;
  Eigen::MatrixXd fState_;
  std::vector<Eigen::VectorXd> mInfo_;

  Eigen::MatrixXd iArmSta_;
  Eigen::MatrixXd fArmSta_;

  // Each col of cfgHs_ denotes a facet (outter_normal^T,point^T)^T
  std::vector<Eigen::MatrixXd> cfgVs_;
  std::vector<Eigen::MatrixXd> cfgHs_;
  Eigen::MatrixXd gdInPs_;
  Eigen::MatrixXd gdAPs_;

  // Piece num for each polytope
  Eigen::VectorXi intervals_;
  // Assignment vector for point in V-polytope
  Eigen::VectorXi idxVs_;
  // Assignment vector for piece in H-polytope
  Eigen::VectorXi idxHs_;

  std::vector<int> inPolysIds_;

  int tN_;
  int dimFreeT_;
  int dimFreeP_;
  int dimFreePWithoutConstraints_;
  int dimArmP_;
  Eigen::VectorXd T_;
  Eigen::MatrixXd innerP_;
  Eigen::MatrixXd innerPA_;

  // Params for constraints
  Eigen::VectorXi cons_;

  // inter parameters
  Config *cfg_;
  std::vector<int> guideIds_; /* guidance points */
  Eigen::MatrixXd guideP_;
  std::vector<int> fixedIds_; /* fixed points */
  Eigen::MatrixXd fixedP_;
  std::vector<int> fixedArmIds_; /* fixed arm points */
  Eigen::MatrixXd fixedArmP_;

  double vMax_;

  // L-BFGS Solver Parameters
  lbfgs::lbfgs_parameter_t lbfgs_params_;

  int iteration_count_phase1_;
  int iteration_count_phase2_;
  int total_iterations_;

  ros::NodeHandle nh_;

  static int progressCallback(void *instance, const double *x, const double *g,
                              const double fx, const double xnorm, const double gnorm,
                              const double step, int n, int k, int ls)
  {
    SE3GCOPTER *obj = static_cast<SE3GCOPTER *>(instance);

    int expected_phase1_size = obj->dimFreeT_ + obj->dimFreeP_ + obj->dimArmP_;
    int expected_phase2_size =
        obj->dimFreeT_ + obj->dimFreePWithoutConstraints_ + obj->dimArmP_;

    if (n == expected_phase1_size)
    {
      obj->iteration_count_phase1_ = k;
    }
    else if (n == expected_phase2_size)
    {
      obj->iteration_count_phase2_ = k;
    }
    obj->total_iterations_ = obj->iteration_count_phase1_ + obj->iteration_count_phase2_;
    return 0;
  }

 private:
  template <typename EIGENVEC>
  static inline void forwardT(const EIGENVEC &t, Eigen::VectorXd &vecT, bool soft,
                              const double &sT, bool c2)
  {
    if (soft)
    {
      if (c2)
      {
        int M = vecT.size();
        for (int i = 0; i < M; i++)
        {
          vecT(i) = t(i) > 0.0 ? ((0.5 * t(i) + 1.0) * t(i) + 1.0)
                               : 1.0 / ((0.5 * t(i) - 1.0) * t(i) + 1.0);
        }
      }
      else
      {
        vecT = t.array().exp();
      }
    }
    else
    {
      if (c2)
      {
        int Ms1 = t.size();
        for (int i = 0; i < Ms1; i++)
        {
          vecT(i) = t(i) > 0.0 ? ((0.5 * t(i) + 1.0) * t(i) + 1.0)
                               : 1.0 / ((0.5 * t(i) - 1.0) * t(i) + 1.0);
        }
        vecT(Ms1) = 0.0;
        vecT /= 1.0 + vecT.sum();
        vecT(Ms1) = 1.0 - vecT.sum();
        vecT *= sT;
      }
      else
      {
        int Ms1 = t.size();
        vecT.head(Ms1) = t.array().exp();
        vecT(Ms1) = 0.0;
        vecT /= 1.0 + vecT.sum();
        vecT(Ms1) = 1.0 - vecT.sum();
        vecT *= sT;
      }
    }
    return;
  }

  template <typename EIGENVEC>
  static inline void backwardT(const Eigen::VectorXd &vecT, EIGENVEC &t, bool soft,
                               bool c2)
  {
    if (soft)
    {
      if (c2)
      {
        int M = vecT.size();
        for (int i = 0; i < M; i++)
        {
          t(i) = vecT(i) > 1.0 ? (sqrt(2.0 * vecT(i) - 1.0) - 1.0)
                               : (1.0 - sqrt(2.0 / vecT(i) - 1.0));
          /*

          t  = 2/[(tau-1)^2+1] tau<0
             = 1/2[(tau+1)^2+1] tau>=0
          */
        }
      }
      else
      {
        t = vecT.array().log();
      }
    }
    else
    {
      if (c2)
      {
        int Ms1 = t.size();
        t = vecT.head(Ms1) / vecT(Ms1);
        for (int i = 0; i < Ms1; i++)
        {
          t(i) = t(i) > 1.0 ? (sqrt(2.0 * t(i) - 1.0) - 1.0)
                            : (1.0 - sqrt(2.0 / t(i) - 1.0));
        }
      }
      else
      {
        int Ms1 = t.size();
        t = (vecT.head(Ms1) / vecT(Ms1)).array().log();
      }
    }
    return;
  }

  template <typename EIGENVEC>
  static inline void forwardP(const Mode mode, const EIGENVEC &p,
                              const Eigen::VectorXi &idVs,
                              const std::vector<Eigen::MatrixXd> &cfgPolyVs,
                              const std::vector<int> &guideIds,
                              const Eigen::MatrixXd &guideP,
                              const std::vector<int> &fixedIds,
                              const Eigen::MatrixXd &fixedP, Eigen::MatrixXd &inP)
  {
    int M = inP.cols();
    Eigen::VectorXd q;
    int j = 0, k, idx;
    for (int i = 0; i < M; i++)
    {
      if (isGuide(i, guideIds) != -1)
      {
        inP.col(i) = guideP.col(isGuide(i, guideIds));
        continue;
      }
      if (isFixed(i, fixedIds) != -1)
      {
        inP.col(i) = fixedP.col(isFixed(i, fixedIds));
        continue;
      }
      idx = idVs(i);
      k = cfgPolyVs[idx].cols() - 1;
      q = 2.0 / (1.0 + p.segment(j, k).squaredNorm()) * p.segment(j, k);
      inP.col(i) =
          cfgPolyVs[idx].rightCols(k) * q.cwiseProduct(q) + cfgPolyVs[idx].col(0);
      j += k;
    }
    return;
  }

  template <typename EIGENVEC>
  static inline void forwardPWithoutConstraints(
      const Mode mode, const EIGENVEC &p, const Eigen::VectorXi &idVs,
      const std::vector<Eigen::MatrixXd> &cfgPolyVs, const std::vector<int> &guideIds,
      const Eigen::MatrixXd &guideP, const std::vector<int> &fixedIds,
      const Eigen::MatrixXd &fixedP, Eigen::MatrixXd &inP)
  {
    int M = inP.cols();
    Eigen::VectorXd q;
    int j = 0, k, idx;
    for (int i = 0; i < M; i++)
    {
      if (isFixed(i, fixedIds) != -1)
      {
        inP.col(i) = fixedP.col(isFixed(i, fixedIds));
        continue;
      }
      idx = idVs(i);
      k = cfgPolyVs[idx].cols() - 1;
      q = 2.0 / (1.0 + p.segment(j, k).squaredNorm()) * p.segment(j, k);
      inP.col(i) =
          cfgPolyVs[idx].rightCols(k) * q.cwiseProduct(q) + cfgPolyVs[idx].col(0);
      j += k;
    }
    return;
  }

  template <typename EIGENVEC>
  static inline void forwardPA(const Mode mode, const EIGENVEC &pA,
                               const std::vector<int> &fixedArmIds,
                               const Eigen::MatrixXd &fixedArmP, Eigen::MatrixXd &inPA)
  {
    int M = inPA.cols();
    int j = 0;
    for (int i = 0; i < M; i++)
    {
      if (isFixedArm(i, fixedArmIds) == -1)
      {
        inPA.col(i)(0) = pA(j++);
        inPA.col(i)(1) = pA(j++);
        inPA.col(i)(2) = pA(j++);
      }
      else
      {
        inPA.col(i)(0) = fixedArmP.col(isFixedArm(i, fixedArmIds))(0);
        inPA.col(i)(1) = fixedArmP.col(isFixedArm(i, fixedArmIds))(1);
        inPA.col(i)(2) = fixedArmP.col(isFixedArm(i, fixedArmIds))(2);
      }
    }
    return;
  }

  static inline double objectiveNLS(void *ptrPOBs, const double *x, double *grad,
                                    const int n)
  {
    const Eigen::MatrixXd &pobs = *(Eigen::MatrixXd *)ptrPOBs;
    Eigen::Map<const Eigen::VectorXd> p(x, n);
    Eigen::Map<Eigen::VectorXd> gradp(grad, n);

    double qnsqr = p.squaredNorm();
    double qnsqrp1 = qnsqr + 1.0;
    double qnsqrp1sqr = qnsqrp1 * qnsqrp1;
    Eigen::VectorXd r = 2.0 / qnsqrp1 * p;

    Eigen::Vector3d delta =
        pobs.rightCols(n) * r.cwiseProduct(r) + pobs.col(1) - pobs.col(0);
    double cost = delta.squaredNorm();
    Eigen::Vector3d gradR3 = 2 * delta;

    Eigen::VectorXd gdr = pobs.rightCols(n).transpose() * gradR3;
    gdr = gdr.array() * r.array() * 2.0;
    gradp = gdr * 2.0 / qnsqrp1 - p * 4.0 * gdr.dot(p) / qnsqrp1sqr;

    return cost;
  }

  template <typename EIGENVEC>
  static inline void backwardP(const Mode mode, const Eigen::MatrixXd &inP,
                               const Eigen::VectorXi &idVs,
                               const std::vector<Eigen::MatrixXd> &cfgPolyVs,
                               const std::vector<int> &guideIds,
                               const std::vector<int> &fixedIds, EIGENVEC &p)
  {
    int M = inP.cols();
    int j = 0, k, idx;

    // Parameters for tiny nonlinear least squares
    double minSqrD;
    lbfgs::lbfgs_parameter_t nls_params;
    lbfgs::lbfgs_load_default_parameters(&nls_params);
    nls_params.g_epsilon = FLT_EPSILON;
    nls_params.max_iterations = 128;

    Eigen::MatrixXd pobs;
    for (int i = 0; i < M; i++)
    {
      if (isGuide(i, guideIds) != -1 || isFixed(i, fixedIds) != -1) continue;
      idx = idVs(i);
      k = cfgPolyVs[idx].cols() - 1;
      p.segment(j, k).setConstant(1.0 / (sqrt(k + 1.0) + 1.0));
      pobs.resize(3, k + 2);
      pobs << inP.col(i), cfgPolyVs[idx];
      lbfgs::lbfgs_optimize(k, p.data() + j, &minSqrD, &SE3GCOPTER::objectiveNLS, nullptr,
                            nullptr, &pobs, &nls_params);
      j += k;
    }
    return;
  }

  template <typename EIGENVEC>
  static inline void backwardPWithoutConstraints(
      const Mode mode, const Eigen::MatrixXd &inP, const Eigen::VectorXi &idVs,
      const std::vector<Eigen::MatrixXd> &cfgPolyVs, const std::vector<int> &guideIds,
      const std::vector<int> &fixedIds, EIGENVEC &p)
  {
    int M = inP.cols();
    int j = 0, k, idx;

    // Parameters for tiny nonlinear least squares
    double minSqrD;
    lbfgs::lbfgs_parameter_t nls_params;
    lbfgs::lbfgs_load_default_parameters(&nls_params);
    nls_params.g_epsilon = FLT_EPSILON;
    nls_params.max_iterations = 128;

    Eigen::MatrixXd pobs;
    for (int i = 0; i < M; i++)
    {
      if (isFixed(i, fixedIds) != -1) continue;
      idx = idVs(i);
      k = cfgPolyVs[idx].cols() - 1;
      p.segment(j, k).setConstant(1.0 / (sqrt(k + 1.0) + 1.0));
      pobs.resize(3, k + 2);
      pobs << inP.col(i), cfgPolyVs[idx];
      lbfgs::lbfgs_optimize(k, p.data() + j, &minSqrD, &SE3GCOPTER::objectiveNLS, nullptr,
                            nullptr, &pobs, &nls_params);
      j += k;
    }
    return;
  }

  template <typename EIGENVEC>
  static inline void backwardPA(const Mode mode, const Eigen::MatrixXd &inPA,
                                const std::vector<int> &fixedArmIds, EIGENVEC &pa)
  {
    int M = inPA.cols();

    int k = 0;
    for (int i = 0; i < M; i++)
    {
      if (isFixedArm(i, fixedArmIds) != -1)
      {
        continue;
      }
      pa(k++) = inPA(0, i);
      pa(k++) = inPA(1, i);
      pa(k++) = inPA(2, i);
    }
    return;
  }

  template <typename EIGENVEC>
  static inline void addLayerTGrad(const Eigen::VectorXd &t, EIGENVEC &gradT, bool soft,
                                   const double &sT, bool c2)
  {
    if (soft)
    {
      if (c2)
      {
        int M = t.size();
        double denSqrt;
        for (int i = 0; i < M; i++)
        {
          if (t(i) > 0)
          {
            gradT(i) *= t(i) + 1.0;
          }
          else
          {
            denSqrt = (0.5 * t(i) - 1.0) * t(i) + 1.0;
            gradT(i) *= (1.0 - t(i)) / (denSqrt * denSqrt);
          }
        }
      }
      else
      {
        int M = t.size();
        gradT.head(M).array() *= t.array().exp();
      }
    }
    else
    {
      if (c2)
      {
        int Ms1 = t.size();
        Eigen::VectorXd gFree = sT * gradT.head(Ms1);
        double gTail = sT * gradT(Ms1);
        Eigen::VectorXd dExpTau(Ms1);
        double expTauSum = 0.0, gFreeDotExpTau = 0.0;
        double denSqrt, expTau;
        for (int i = 0; i < Ms1; i++)
        {
          if (t(i) > 0)
          {
            expTau = (0.5 * t(i) + 1.0) * t(i) + 1.0;
            dExpTau(i) = t(i) + 1.0;
            expTauSum += expTau;
            gFreeDotExpTau += expTau * gFree(i);
          }
          else
          {
            denSqrt = (0.5 * t(i) - 1.0) * t(i) + 1.0;
            expTau = 1.0 / denSqrt;
            dExpTau(i) = (1.0 - t(i)) / (denSqrt * denSqrt);
            expTauSum += expTau;
            gFreeDotExpTau += expTau * gFree(i);
          }
        }
        denSqrt = expTauSum + 1.0;
        gradT.head(Ms1) =
            (gFree.array() - gTail) * dExpTau.array() / denSqrt -
            (gFreeDotExpTau - gTail * expTauSum) * dExpTau.array() / (denSqrt * denSqrt);
        gradT(Ms1) = 0.0;
      }
      else
      {
        int Ms1 = t.size();
        Eigen::VectorXd gFree = sT * gradT.head(Ms1);
        double gTail = sT * gradT(Ms1);
        Eigen::VectorXd expTau = t.array().exp();
        double expTauSum = expTau.sum();
        double denom = expTauSum + 1.0;
        gradT.head(Ms1) =
            (gFree.array() - gTail) * expTau.array() / denom -
            (gFree.dot(expTau) - gTail * expTauSum) * expTau.array() / (denom * denom);
        gradT(Ms1) = 0.0;
      }
    }
    return;
  }

  template <typename EIGENVEC_0, typename EIGENVEC_1>
  static inline void addLayerPGrad(const Mode mode, EIGENVEC_0 &p,
                                   const Eigen::VectorXi &idVs,
                                   const std::vector<Eigen::MatrixXd> &cfgPolyVs,
                                   const std::vector<int> &guideIds,
                                   const Eigen::MatrixXd &gradInPs,
                                   const std::vector<int> &fixedIds,
                                   const Eigen::MatrixXd &fixedP, EIGENVEC_1 &grad)
  {
    int M = gradInPs.cols();

    int j = 0, k, idx;
    double qnsqr, qnsqrp1, qnsqrp1sqr;
    Eigen::VectorXd q, r, gdr;
    for (int i = 0; i < M; i++)
    {
      if (isGuide(i, guideIds) != -1 || isFixed(i, fixedIds) != -1) continue;
      idx = idVs(i);
      k = cfgPolyVs[idx].cols() - 1;

      q = p.segment(j, k);
      qnsqr = q.squaredNorm();
      qnsqrp1 = qnsqr + 1.0;
      qnsqrp1sqr = qnsqrp1 * qnsqrp1;
      r = 2.0 / qnsqrp1 * q;
      gdr = cfgPolyVs[idx].rightCols(k).transpose() * gradInPs.col(i);
      gdr = gdr.array() * r.array() * 2.0;

      grad.segment(j, k) = gdr * 2.0 / qnsqrp1 - q * 4.0 * gdr.dot(q) / qnsqrp1sqr;

      j += k;
    }

    return;
  }

  template <typename EIGENVEC_0, typename EIGENVEC_1>
  static inline void addLayerPGradWithoutConstraints(
      const Mode mode, EIGENVEC_0 &p, const Eigen::VectorXi &idVs,
      const std::vector<Eigen::MatrixXd> &cfgPolyVs, const std::vector<int> &guideIds,
      const Eigen::MatrixXd &gradInPs, const std::vector<int> &fixedIds,
      const Eigen::MatrixXd &fixedP, EIGENVEC_1 &grad)
  {
    int M = gradInPs.cols();

    int j = 0, k, idx;
    double qnsqr, qnsqrp1, qnsqrp1sqr;
    Eigen::VectorXd q, r, gdr;
    for (int i = 0; i < M; i++)
    {
      if (isFixed(i, fixedIds) != -1) continue;
      idx = idVs(i);
      k = cfgPolyVs[idx].cols() - 1;

      q = p.segment(j, k);
      qnsqr = q.squaredNorm();
      qnsqrp1 = qnsqr + 1.0;
      qnsqrp1sqr = qnsqrp1 * qnsqrp1;
      r = 2.0 / qnsqrp1 * q;
      gdr = cfgPolyVs[idx].rightCols(k).transpose() * gradInPs.col(i);
      gdr = gdr.array() * r.array() * 2.0;

      grad.segment(j, k) = gdr * 2.0 / qnsqrp1 - q * 4.0 * gdr.dot(q) / qnsqrp1sqr;

      j += k;
    }

    return;
  }

  template <typename EIGENVEC_0, typename EIGENVEC_1>
  static inline void addLayerPAGrad(const Mode mode, EIGENVEC_0 &pA,
                                    const Eigen::MatrixXd &gradInPA,
                                    const std::vector<int> &fixedArmIds, EIGENVEC_1 &grad)
  {
    int M = gradInPA.cols();

    int k = 0;
    for (int i = 0; i < M; i++)
    {
      if (isFixedArm(i, fixedArmIds) != -1)
      {
        continue;
      }
      grad(k++) = gradInPA(0, i);
      grad(k++) = gradInPA(1, i);
      grad(k++) = gradInPA(2, i);
    }

    return;
  }

  static inline void mergeToCoarseGradT(const Eigen::VectorXi &intervs,
                                        Eigen::VectorXd &fineGdT)
  {
    int M = intervs.size();
    int offset = 0;
    int inverv;
    for (int i = 0; i < M; i++)
    {
      inverv = intervs(i);
      fineGdT(i) = fineGdT.segment(offset, inverv).mean();
      offset += inverv;
    }
    return;
  }

  static inline double objectiveFunc(void *ptrObj, const double *x, double *grad,
                                     const int n)
  {
    SE3GCOPTER &obj = *(SE3GCOPTER *)ptrObj;
    const int dimT = obj.dimFreeT_;
    const int dimP = obj.dimFreeP_;
    const int dimAP = obj.dimArmP_;
    const double rh = obj.rho_;
    Eigen::Map<const Eigen::VectorXd> t(x, dimT);
    Eigen::Map<const Eigen::VectorXd> p(x + dimT, dimP);
    Eigen::Map<const Eigen::VectorXd> pA(x + dimT + dimP, dimAP);
    Eigen::Map<Eigen::VectorXd> gradt(grad, dimT);
    Eigen::VectorXd proxyGradT(dimT);
    Eigen::VectorXd proxyGradTA(dimT);
    Eigen::Map<Eigen::VectorXd> gradp(grad + dimT, dimP);
    Eigen::Map<Eigen::VectorXd> gradpA(grad + dimT + dimP, dimAP);
    Eigen::MatrixXd gdC_ARM(6 * dimT, 3);
    Eigen::MatrixXd C_ARM(6 * dimT, 3);
    double cost = 0;
    double arm_cost = 0;

    /* Generate flying base trajectory */
    forwardT(t, obj.T_, obj.softT_, obj.sumT_, obj.c2dfm_);
    forwardP(obj.mode_, p, obj.idxVs_, obj.cfgVs_, obj.guideIds_, obj.guideP_,
             obj.fixedIds_, obj.fixedP_, obj.innerP_);
    forwardPA(obj.mode_, pA, obj.fixedArmIds_, obj.fixedArmP_, obj.innerPA_);

    /* Generate trajectory */
    obj.jerkOpt_.generate(obj.innerP_, obj.T_);
    obj.armOpt_.generate(obj.innerPA_, obj.T_);
    C_ARM = obj.armOpt_.getCoeffs();

    /* evaluate the penalty */
    obj.jerkOpt_.evalBaseTrajCostGrad(obj.mode_, obj.cons_, obj.idxHs_, obj.cfgHs_, cost,
                                      proxyGradT, obj.gdInPs_, obj.mInfo_,
                                      obj.inPolysIds_, obj.cfg_, C_ARM, gdC_ARM);
    obj.armOpt_.evalArmTrajCostGrad(obj.mode_, obj.cons_, obj.workspaceModel_, obj.gdAPs_,
                                    proxyGradTA, arm_cost, obj.cfg_, obj.mInfo_,
                                    obj.inPolysIds_, gdC_ARM);
    cost += rh * obj.T_.sum();
    proxyGradT.array() += rh;
    proxyGradT.array() += proxyGradTA.array();
    addLayerPAGrad(obj.mode_, pA, obj.gdAPs_, obj.fixedArmIds_, gradpA);
    mergeToCoarseGradT(obj.intervals_, proxyGradT);
    // grad of T
    // T,P->tau kesi
    addLayerTGrad(t, proxyGradT, obj.softT_, obj.sumT_, obj.c2dfm_);
    addLayerPGrad(obj.mode_, p, obj.idxVs_, obj.cfgVs_, obj.guideIds_, obj.gdInPs_,
                  obj.fixedIds_, obj.fixedP_, gradp);
    gradt = proxyGradT.head(dimT);
    if (obj.cfg_->printCost) std::cout << "cost = " << cost + arm_cost << std::endl;
    return cost + arm_cost;
  }

  static inline double objectiveFuncWithoutConstraints(void *ptrObj, const double *x,
                                                       double *grad, const int n)
  {
    SE3GCOPTER &obj = *(SE3GCOPTER *)ptrObj;
    const int dimT = obj.dimFreeT_;
    const int dimP = obj.dimFreePWithoutConstraints_;
    const int dimAP = obj.dimArmP_;
    const double rh = obj.rho_;
    Eigen::Map<const Eigen::VectorXd> t(x, dimT);
    Eigen::Map<const Eigen::VectorXd> p(x + dimT, dimP);
    Eigen::Map<const Eigen::VectorXd> pA(x + dimT + dimP, dimAP);
    Eigen::Map<Eigen::VectorXd> gradt(grad, dimT);
    Eigen::VectorXd proxyGradT(dimT);
    Eigen::VectorXd proxyGradTA(dimT);
    Eigen::Map<Eigen::VectorXd> gradp(grad + dimT, dimP);
    Eigen::Map<Eigen::VectorXd> gradpA(grad + dimT + dimP, dimAP);
    Eigen::MatrixXd gdC_ARM(6 * dimT, 3);
    Eigen::MatrixXd C_ARM(6 * dimT, 3);
    double cost = 0;
    double arm_cost = 0;

    /* Generate flying base trajectory */
    forwardT(t, obj.T_, obj.softT_, obj.sumT_, obj.c2dfm_);
    forwardPWithoutConstraints(obj.mode_, p, obj.idxVs_, obj.cfgVs_, obj.guideIds_,
                               obj.guideP_, obj.fixedIds_, obj.fixedP_, obj.innerP_);
    forwardPA(obj.mode_, pA, obj.fixedArmIds_, obj.fixedArmP_, obj.innerPA_);

    /* Generate trajectory */
    obj.jerkOpt_.generate(obj.innerP_, obj.T_);
    obj.armOpt_.generate(obj.innerPA_, obj.T_);
    C_ARM = obj.armOpt_.getCoeffs();

    /* evaluate the penalty */
    /* cf. minco_base.h */
    obj.jerkOpt_.evalBaseTrajCostGrad(obj.mode_, obj.cons_, obj.idxHs_, obj.cfgHs_, cost,
                                      proxyGradT, obj.gdInPs_, obj.mInfo_,
                                      obj.inPolysIds_, obj.cfg_, C_ARM, gdC_ARM);
    /* cf. minco_arm.h */
    obj.armOpt_.evalArmTrajCostGrad(obj.mode_, obj.cons_, obj.workspaceModel_, obj.gdAPs_,
                                    proxyGradTA, arm_cost, obj.cfg_, obj.mInfo_,
                                    obj.inPolysIds_, gdC_ARM);
    cost += rh * obj.T_.sum();
    proxyGradT.array() += rh;
    proxyGradT.array() += proxyGradTA.array();
    addLayerPAGrad(obj.mode_, pA, obj.gdAPs_, obj.fixedArmIds_, gradpA);
    mergeToCoarseGradT(obj.intervals_, proxyGradT);
    // grad of T
    // T,P->tau kesi
    addLayerTGrad(t, proxyGradT, obj.softT_, obj.sumT_, obj.c2dfm_);
    addLayerPGradWithoutConstraints(obj.mode_, p, obj.idxVs_, obj.cfgVs_, obj.guideIds_,
                                    obj.gdInPs_, obj.fixedIds_, obj.fixedP_, gradp);
    gradt = proxyGradT.head(dimT);

    if (obj.cfg_->printCost) std::cout << "cost = " << cost + arm_cost << std::endl;
    return cost + arm_cost;
  }

 public:
  inline void gridMesh(const Eigen::Matrix3d &iState, const Eigen::Matrix3d &fState,
                       const std::vector<Eigen::MatrixXd> &cfgPolyVs,
                       const double &gridResolution, Eigen::VectorXi &intervals_Vec) const
  {
    int M = intervals_Vec.size();

    int curInterval, k;
    Eigen::Vector3d lastP, curP;
    curP = iState.col(0);
    for (int i = 0; i < M - 1; i++)
    {
      lastP = curP;
      k = cfgPolyVs[2 * i + 1].cols() - 1;
      curP = cfgPolyVs[2 * i + 1].rightCols(k).rowwise().sum() / (1.0 + k) +
             cfgPolyVs[2 * i + 1].col(0);
      curInterval = ceil((curP - lastP).norm() / gridResolution);
      intervals_Vec(i) = curInterval > 0 ? curInterval : 1;
    }
    lastP = curP;

    curP = fState.col(0);
    curInterval = ceil((curP - lastP).norm() / gridResolution);
    intervals_Vec(M - 1) = curInterval > 0 ? curInterval : 1;

    return;
  }

  inline bool extractVs(const std::vector<Eigen::MatrixXd> &hPs,
                        std::vector<Eigen::MatrixXd> &vPs) const
  {
    const int M = hPs.size() - 1;

    vPs.clear();
    vPs.reserve(2 * M + 1);

    int nv;
    Eigen::MatrixXd curIH, curIV, curIOB;
    for (int i = 0; i < M; i++)
    {
      if (!geoutils::enumerateVs(hPs[i], curIV))
      {
        return false;
      }
      nv = curIV.cols();
      curIOB.resize(3, nv);
      curIOB << curIV.col(0), curIV.rightCols(nv - 1).colwise() - curIV.col(0);
      vPs.push_back(curIOB);

      curIH.resize(6, hPs[i].cols() + hPs[i + 1].cols());
      curIH << hPs[i], hPs[i + 1];
      if (!geoutils::enumerateVs(curIH, curIV))
      {
        return false;
      }
      nv = curIV.cols();
      curIOB.resize(3, nv);
      curIOB << curIV.col(0), curIV.rightCols(nv - 1).colwise() - curIV.col(0);
      vPs.push_back(curIOB);
    }

    if (!geoutils::enumerateVs(hPs.back(), curIV))
    {
      return false;
    }
    nv = curIV.cols();
    curIOB.resize(3, nv);
    curIOB << curIV.col(0), curIV.rightCols(nv - 1).colwise() - curIV.col(0);
    vPs.push_back(curIOB);

    return true;
  }

  inline bool extractV(const Eigen::MatrixXd &hP, Eigen::MatrixXd &vP) const
  {
    int nv;
    Eigen::MatrixXd curIV, curIOB;
    if (!geoutils::enumerateVs(hP, curIV))
    {
      return false;
    }
    nv = curIV.cols();
    curIOB.resize(3, nv);
    curIOB << curIV.col(0), curIV.rightCols(nv - 1).colwise() - curIV.col(0);
    vP = curIOB;
    return true;
  }

  template <typename EIGENVEC>
  inline void setInitArm(const Eigen::VectorXd &T, const std::vector<int> &fixedArmIds,
                         const Eigen::MatrixXd &fixedArmP, EIGENVEC &pa)
  {
    int cols = T.size() - 1;
    int k = 0;
    for (int i = 0; i < cols; i++)
    {
      if (isFixedArm(i, fixedArmIds) != -1)
      {
        pa(k++) = fixedArmP.col(isFixedArm(i, fixedArmIds))(0);
        pa(k++) = fixedArmP.col(isFixedArm(i, fixedArmIds))(1);
        pa(k++) = fixedArmP.col(isFixedArm(i, fixedArmIds))(2);
      }
      else
      {
        pa(k++) = 0.0;
        pa(k++) = 0.0;
        pa(k++) = -cfg_->boundArmZ;
      }
    }
  }

  inline void setGuidance(const Mode mode, const std::vector<Eigen::VectorXd> &mInfo,
                          const std::vector<int> &interPolysIds, Eigen::MatrixXd &guideP,
                          std::vector<int> &guideIds)
  {
    int guide_num = 0;
    for (int i = 0; i < mInfo.size(); i++)
    {
      if (mInfo[i](0) == 0.0)
      {
        guide_num++;
      }
    }
    guideP.resize(3, guide_num);
    int guide_idx = 0;
    for (int i = 0; i < mInfo.size(); i++)
    {
      if (mInfo[i](0) == 0.0)
      {
        guideP.col(guide_idx) = mInfo[i].segment(1, 3);
        guideIds.push_back(interPolysIds[i]);
        guide_idx++;
      }
    }
  }

  inline void setFixed(const Mode mode, const std::vector<Eigen::VectorXd> &mInfo,
                       const std::vector<int> &interPolysIds, Eigen::MatrixXd &fixedP,
                       std::vector<int> &fixedIds)
  {
    int fixed_num = 0;
    for (int i = 0; i < mInfo.size(); i++)
    {
      if (mInfo[i](0) == 1.0 || mInfo[i](0) == 3.0)
      {
        fixed_num++;
      }
    }
    fixedP.resize(3, fixed_num);
    int fixed_idx = 0;
    for (int i = 0; i < mInfo.size(); i++)
    {
      if (mInfo[i](0) == 1.0 || mInfo[i](0) == 3.0)
      {
        fixedP.col(fixed_idx) = mInfo[i].segment(1, 3);
        fixedIds.push_back(interPolysIds[i]);
        fixed_idx++;
      }
    }
  }

  inline void setFixedArm(const Mode mode, const std::vector<Eigen::VectorXd> &mInfo,
                          const std::vector<int> &interPolysIds,
                          Eigen::MatrixXd &fixedArmP, std::vector<int> &fixedArmIds)
  {
    int fixed_num = 0;
    for (int i = 0; i < mInfo.size(); i++)
    {
      if (mInfo[i](0) == 3.0)
      {
        fixed_num++;
      }
    }
    fixedArmP.resize(3, fixed_num);
    int fixed_idx = 0;
    for (int i = 0; i < mInfo.size(); i++)
    {
      if (mInfo[i](0) == 3.0)
      {
        fixedArmP.col(fixed_idx) = mInfo[i].segment(4, 3);
        fixedArmIds.push_back(interPolysIds[i]);
        fixed_idx++;
      }
    }
  }

  inline bool setup(const Mode &mod, const Eigen::MatrixXd &iniState,
                    const std::vector<Eigen::VectorXd> &interInfo,
                    const Eigen::MatrixXd &finState,
                    const std::vector<int> &interPolysIds,
                    const std::vector<Eigen::MatrixXd> &cfgPolyHs, const double &gridRes,
                    Config &config, ros::NodeHandle &nh)
  {
    nh_ = nh;
    cfg_ = &config;
    c2dfm_ = cfg_->useC2Diffeo;
    softT_ = cfg_->rho > 0;
    mode_ = mod;

    if (softT_)
    {
      rho_ = cfg_->rho;
      sumT_ = 1.0;
    }

    iState_ = iniState;
    fState_ = finState;

    mInfo_ = interInfo;

    iArmSta_.setZero(3, 3);
    fArmSta_.setZero(3, 3);
    iArmSta_.col(0) = Eigen::Vector3d(0, 0, -cfg_->boundArmZ);
    fArmSta_.col(0) = Eigen::Vector3d(0, 0, -cfg_->boundArmZ);

    guideIds_.clear();
    fixedIds_.clear();
    fixedArmIds_.clear();
    inPolysIds_ = interPolysIds;
    setGuidance(mode_, mInfo_, inPolysIds_, guideP_, guideIds_);
    setFixed(mode_, mInfo_, inPolysIds_, fixedP_, fixedIds_);
    setFixedArm(mode_, mInfo_, inPolysIds_, fixedArmP_, fixedArmIds_);

    cfgHs_ = cfgPolyHs;
    tN_ = cfgHs_.size();
    for (int i = 0; i < tN_; i++)
    {
      cfgHs_[i].topRows<3>().colwise().normalize();  // 6*n
    }
    if (!extractVs(cfgHs_, cfgVs_))
    {
      ROS_INFO("Failed to extract vertices from cfgHs_");
      return false;
    }

    intervals_.resize(tN_);
    gridMesh(iState_, fState_, cfgVs_, gridRes, intervals_);  // cfgVs_
                                                              // dimension:2n-1
    tN_ = intervals_.sum();
    cons_.resize(tN_);
    cons_.setConstant(cfg_->qdIntervals);

    idxVs_.resize(tN_ - 1);
    idxHs_.resize(tN_);
    dimFreeT_ = softT_ ? tN_ : tN_ - 1;  // softT_ is true tau size is n

    dimFreeP_ = 0;
    dimFreePWithoutConstraints_ = 0;
    int offset = 0, interval;
    for (int i = 0; i < tN_; i++)
    {
      if (i < tN_ - 1)
      {
        idxVs_(offset) = 2 * i + 1;
        if (isFixed(i, fixedIds_) == -1)
        {
          dimFreePWithoutConstraints_ += cfgVs_[2 * i + 1].cols() - 1;
          if (isGuide(i, guideIds_) == -1)
          {
            dimFreeP_ += cfgVs_[2 * i + 1].cols() - 1;
          }
        }
      }
      idxHs_(offset) = i;
      offset++;
    }

    dimArmP_ = 3 * (dimFreeT_ - 1 - fixedArmIds_.size());

    vMax_ = cfg_->velMax;
    // Make a legal initial speed
    double tempNorm;
    tempNorm = iState_.col(1).norm();
    iState_.col(1) *= tempNorm > vMax_ ? (vMax_ / tempNorm) : 1.0;
    tempNorm = fState_.col(1).norm();
    fState_.col(1) *= tempNorm > vMax_ ? (vMax_ / tempNorm) : 1.0;

    lbfgs::lbfgs_load_default_parameters(&lbfgs_params_);
    T_.resize(tN_);
    innerP_.resize(3, tN_ - 1);
    innerPA_.resize(3, tN_ - 1);
    gdInPs_.resize(3, tN_ - 1);
    gdAPs_.resize(3, tN_ - 1);
    jerkOpt_.reset(iState_, fState_, tN_);
    armOpt_.reset(iArmSta_, fArmSta_, tN_);

    return true;
  }

  inline void setInitial(const Mode mode, const Config *cfg,
                         const Eigen::MatrixXd &iState, const Eigen::MatrixXd &guideP,
                         const Eigen::MatrixXd &fixedP, const Eigen::MatrixXd &fState,
                         const std::vector<Eigen::MatrixXd> &cfgPolyVs,
                         const Eigen::VectorXi &intervs, const vector<int> &guideIds,
                         const vector<int> &fixedIds, Eigen::VectorXd &vecT,
                         Eigen::MatrixXd &vecInP) const
  {
    constexpr double maxSpeedForAllocatiion = 10.0;

    int M = vecT.size();
    Eigen::Vector3d lastP, curP, delta;
    int offset, interv, k;

    offset = 0;

    curP = iState.col(0);
    for (int i = 0; i < M - 1; i++)
    {
      lastP = curP;
      interv = intervs(i);
      k = cfgPolyVs[2 * i + 1].cols() - 1;
      if (isGuide(i, guideIds) == -1 && isFixed(i, fixedIds) == -1)
      {
        curP = cfgPolyVs[2 * i + 1].rightCols(k).rowwise().sum() / (1.0 + k) +
               cfgPolyVs[2 * i + 1].col(0);
      }
      else if (isGuide(i, guideIds) != -1)
      {
        curP = guideP.col(isGuide(i, guideIds));
      }
      else
      {
        curP = fixedP.col(isFixed(i, fixedIds));
      }
      delta = curP - lastP;
      // vecT(i) = delta.norm() / std::min(vMax, maxSpeedForAllocatiion);
      vecT(i) = std::max(delta.norm() / cfg->velMax * cfg->initVel, 0.1);
      delta /= interv;
      for (int j = 0; j < interv; j++)
      {
        vecInP.col(offset++) = (j + 1) * delta + lastP;
      }
    }
    interv = intervs(M - 1);
    lastP = curP;
    curP = fState.col(0);
    delta = curP - lastP;
    // vecT(M - 1) = delta.norm() / std::min(vMax, maxSpeedForAllocatiion);
    vecT(M - 1) = delta.norm() / cfg->velMax * cfg->initVel;
    delta /= interv;
    for (int j = 0; j < interv - 1; j++)
    {
      vecInP.col(offset++) = (j + 1) * delta + lastP;
    }
    return;
  }

  inline static int isGuide(const int &id, const std::vector<int> &guideIds)
  {
    for (int i = 0; i < guideIds.size(); i++)
    {
      if (guideIds[i] == id) return i;
    }
    return -1;
  }
  inline static int isFixed(const int &id, const std::vector<int> &fixedIds)
  {
    for (int i = 0; i < fixedIds.size(); i++)
    {
      if (fixedIds[i] == id) return i;
    }
    return -1;
  }
  inline static int isFixedArm(const int &id, const std::vector<int> &fixedArmIds)
  {
    for (int i = 0; i < fixedArmIds.size(); i++)
    {
      if (fixedArmIds[i] == id) return i;
    }
    return -1;
  }

  inline double optimize(Trajectory<5> &traj, Trajectory<5> &arm_traj,
                         Trajectory<5> &traj_temp, Trajectory<5> &arm_traj_temp,
                         Eigen::MatrixXd &iP, double &grasp_time,
                         std::vector<double> &logs)
  {
    /* initial the variables */
    double *x = new double[dimFreeT_ + dimFreeP_ + dimArmP_];
    Eigen::Map<Eigen::VectorXd> t(x, dimFreeT_);
    Eigen::Map<Eigen::VectorXd> p(x + dimFreeT_, dimFreeP_);
    Eigen::Map<Eigen::VectorXd> pa(x + dimFreeT_ + dimFreeP_, dimArmP_);

    double *x_ = new double[dimFreeT_ + dimFreePWithoutConstraints_ + dimArmP_];
    Eigen::Map<Eigen::VectorXd> t_(x_, dimFreeT_);
    Eigen::Map<Eigen::VectorXd> p_(x_ + dimFreeT_, dimFreePWithoutConstraints_);
    Eigen::Map<Eigen::VectorXd> pa_(x_ + dimFreeT_ + dimFreePWithoutConstraints_,
                                    dimArmP_);

    /* initialize the waypoints and segment T */
    setInitial(mode_, cfg_, iState_, guideP_, fixedP_, fState_, cfgVs_, intervals_,
               guideIds_, fixedIds_, T_, innerP_);
    setInitArm(T_, fixedArmIds_, fixedArmP_, innerPA_);
    ROS_INFO("[SE3GCOPTER]: set initial done");

    backwardT(T_, t, softT_, c2dfm_);
    backwardP(mode_, innerP_, idxVs_, cfgVs_, guideIds_, fixedIds_, p);
    backwardPA(mode_, innerPA_, fixedArmIds_, pa);
    ROS_INFO("[SE3GCOPTER]: backward done");

    /* initial the arm waypoints */
    double minObjectivePenalty1 = -1.0;
    double minObjectivePenalty2 = -1.0;
    lbfgs_params_.mem_size = 128;
    lbfgs_params_.past = 3;
    lbfgs_params_.min_step = 1e-32;
    lbfgs_params_.g_epsilon = 1e-32;
    lbfgs_params_.s_curv_coeff = cfg_->sCurvCoeff;
    lbfgs_params_.delta = cfg_->optRelTol1;
    lbfgs_params_.line_search_type = 0;
    int ret1 = 1, ret2 = 1;
    bool uable_to_grasp = false;
    double start_time = ros::Time::now().toSec();
    double discard_time = 0.0;

    /* the most important function of MINCO optimizer is the objectiveFunc!!!!!! */
    ret1 =
        lbfgs::lbfgs_optimize(dimFreeT_ + dimFreeP_ + dimArmP_, x, &minObjectivePenalty1,
                              &SE3GCOPTER::objectiveFunc, nullptr,
                              &SE3GCOPTER::progressCallback, this, &lbfgs_params_);
    forwardT(t, T_, softT_, sumT_, c2dfm_);
    forwardP(mode_, p, idxVs_, cfgVs_, guideIds_, guideP_, fixedIds_, fixedP_, innerP_);
    forwardPA(mode_, pa, fixedArmIds_, fixedArmP_, innerPA_);
    {
      jerkOpt_.generate(innerP_, T_);
      traj_temp = jerkOpt_.getTraj();
      armOpt_.generate(innerPA_, T_);
      arm_traj_temp = armOpt_.getTraj();
    }
    backwardPWithoutConstraints(mode_, innerP_, idxVs_, cfgVs_, guideIds_, fixedIds_, p_);
    backwardPA(mode_, innerPA_, fixedArmIds_, pa_);
    backwardT(T_, t_, softT_, c2dfm_);

    /* second stage optimization */
    lbfgs_params_.delta = cfg_->optRelTol2;
    if (cfg_->multiLayerOpt)
    {
      discard_time = ros::Time::now().toSec();
      ret2 = lbfgs::lbfgs_optimize(dimFreeT_ + dimFreePWithoutConstraints_ + dimArmP_, x_,
                                   &minObjectivePenalty2,
                                   &SE3GCOPTER::objectiveFuncWithoutConstraints, nullptr,
                                   &SE3GCOPTER::progressCallback, this, &lbfgs_params_);
    }

    // ROS_INFO("first minObjectivePenalty = %f", minObjectivePenalty1);
    if (cfg_->multiLayerOpt)
    {
      // ROS_INFO("second minObjectivePenalty = %f", minObjectivePenalty2);
    }

    forwardT(t_, T_, softT_, sumT_, c2dfm_);
    forwardPWithoutConstraints(mode_, p_, idxVs_, cfgVs_, guideIds_, guideP_, fixedIds_,
                               fixedP_, innerP_);
    forwardPA(mode_, pa_, fixedArmIds_, fixedArmP_, innerPA_);

    /* for Debugging */
    iP = innerP_;

    jerkOpt_.generate(innerP_, T_);
    traj = jerkOpt_.getTraj();
    armOpt_.generate(innerPA_, T_);
    arm_traj = armOpt_.getTraj();

    if (cfg_->multiLayerOpt)
    {
      return minObjectivePenalty2;
    }
    else
    {
      return minObjectivePenalty1;
    }
  }
};

WorkspaceProbabilityModel SE3GCOPTER::workspaceModel_;

#endif
