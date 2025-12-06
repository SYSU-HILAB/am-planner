#ifndef MINCO_BASE_H
#define MINCO_BASE_H
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>

#include "traj_opt/config.h"
#include "trajectory.h"

class MINCO_S3_BASE
{
 public:
  MINCO_S3_BASE() = default;
  ~MINCO_S3_BASE() { A.destroy(); }
  // my
  // double compute_time = 0;
 private:
  int N;
  Eigen::Matrix3d headPVA;
  Eigen::Matrix3d tailPVA;
  Eigen::Matrix3d graspPVA;
  int grasp_pt = -1;
  BandedSystem A;
  Eigen::MatrixXd b;

  // Temp variables
  Eigen::VectorXd T1;
  Eigen::VectorXd T2;
  Eigen::VectorXd T3;
  Eigen::VectorXd T4;
  Eigen::VectorXd T5;
  Eigen::MatrixXd gdC;

  struct autoDiffParam
  {
    double gravAcc;
    double mass;
    Eigen::Vector3d Fext;
    bool hasFext;
    Eigen::Vector3d objPos;
    Eigen::Vector3d BPosD;
    Eigen::VectorXd gWeight;
    double EEDownVel;
    Eigen::Vector3d xyzIsCons;
    Eigen::Vector3d xyzConsVals;
    Eigen::Vector3d xyzVelIsCons;
    Eigen::Vector3d xyzConsVelVals;
  };

 private:
  template <typename EIGENVEC>
  inline void addGradJbyT(EIGENVEC &gdT) const
  {
    for (int i = 0; i < N; i++)
    {
      gdT(i) += 36.0 * b.row(6 * i + 3).squaredNorm() +
                288.0 * b.row(6 * i + 4).dot(b.row(6 * i + 3)) * T1(i) +
                576.0 * b.row(6 * i + 4).squaredNorm() * T2(i) +
                720.0 * b.row(6 * i + 5).dot(b.row(6 * i + 3)) * T2(i) +
                2880.0 * b.row(6 * i + 5).dot(b.row(6 * i + 4)) * T3(i) +
                3600.0 * b.row(6 * i + 5).squaredNorm() * T4(i);
    }
    return;
  }

  template <typename EIGENMAT>
  inline void addGradJbyC(EIGENMAT &gdC) const
  {
    for (int i = 0; i < N; i++)
    {
      gdC.row(6 * i + 5) += 240.0 * b.row(6 * i + 3) * T3(i) +
                            720.0 * b.row(6 * i + 4) * T4(i) +
                            1440.0 * b.row(6 * i + 5) * T5(i);
      gdC.row(6 * i + 4) += 144.0 * b.row(6 * i + 3) * T2(i) +
                            384.0 * b.row(6 * i + 4) * T3(i) +
                            720.0 * b.row(6 * i + 5) * T4(i);
      gdC.row(6 * i + 3) += 72.0 * b.row(6 * i + 3) * T1(i) +
                            144.0 * b.row(6 * i + 4) * T2(i) +
                            240.0 * b.row(6 * i + 5) * T3(i);
    }
    return;
  }

  inline void solveAdjGradC(Eigen::MatrixXd &gdC) const
  {
    A.solveAdj(gdC);
    return;
  }

  template <typename EIGENVEC>
  inline void addPropCtoT(const Eigen::MatrixXd &adjGdC, EIGENVEC &gdT) const
  {
    Eigen::MatrixXd B1(6, 3), B2(3, 3);

    Eigen::RowVector3d negVel, negAcc, negJer, negSnp, negCrk;

    for (int i = 0; i < N - 1; i++)
    {
      negVel = -(b.row(i * 6 + 1) + 2.0 * T1(i) * b.row(i * 6 + 2) +
                 3.0 * T2(i) * b.row(i * 6 + 3) + 4.0 * T3(i) * b.row(i * 6 + 4) +
                 5.0 * T4(i) * b.row(i * 6 + 5));
      negAcc = -(2.0 * b.row(i * 6 + 2) + 6.0 * T1(i) * b.row(i * 6 + 3) +
                 12.0 * T2(i) * b.row(i * 6 + 4) + 20.0 * T3(i) * b.row(i * 6 + 5));
      negJer = -(6.0 * b.row(i * 6 + 3) + 24.0 * T1(i) * b.row(i * 6 + 4) +
                 60.0 * T2(i) * b.row(i * 6 + 5));
      negSnp = -(24.0 * b.row(i * 6 + 4) + 120.0 * T1(i) * b.row(i * 6 + 5));
      negCrk = -120.0 * b.row(i * 6 + 5);

      B1 << negSnp, negCrk, negVel, negVel, negAcc, negJer;

      gdT(i) += B1.cwiseProduct(adjGdC.block<6, 3>(6 * i + 3, 0)).sum();
    }

    negVel = -(b.row(6 * N - 5) + 2.0 * T1(N - 1) * b.row(6 * N - 4) +
               3.0 * T2(N - 1) * b.row(6 * N - 3) + 4.0 * T3(N - 1) * b.row(6 * N - 2) +
               5.0 * T4(N - 1) * b.row(6 * N - 1));
    negAcc = -(2.0 * b.row(6 * N - 4) + 6.0 * T1(N - 1) * b.row(6 * N - 3) +
               12.0 * T2(N - 1) * b.row(6 * N - 2) + 20.0 * T3(N - 1) * b.row(6 * N - 1));
    negJer = -(6.0 * b.row(6 * N - 3) + 24.0 * T1(N - 1) * b.row(6 * N - 2) +
               60.0 * T2(N - 1) * b.row(6 * N - 1));

    B2 << negVel, negAcc, negJer;

    gdT(N - 1) += B2.cwiseProduct(adjGdC.block<3, 3>(6 * N - 3, 0)).sum();

    return;
  }

  template <typename EIGENMAT>
  inline void addPropCtoP(const Eigen::MatrixXd &adjGdC, EIGENMAT &gdInP) const
  {
    for (int i = 0; i < N - 1; i++)
    {
      gdInP.col(i) += adjGdC.row(6 * i + 5).transpose();
    }
    return;
  }

  static double smoothedL1(const double &x, double &grad)
  {
    static double mu = 0.01;
    if (x < 0.0)
    {
      return 0.0;
    }
    else if (x > mu)
    {
      grad = 1.0;
      return x - 0.5 * mu;
    }
    else
    {
      const double xdmu = x / mu;
      const double sqrxdmu = xdmu * xdmu;
      const double mumxd2 = mu - 0.5 * x;
      grad = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
      return mumxd2 * sqrxdmu * xdmu;
    }
  }
  static double smoothed01(const double &x, double &grad, const double &mu = 0.01)
  {
    static double mu4 = mu * mu * mu * mu;
    static double mu4_1 = 1.0 / mu4;
    if (x < -mu)
    {
      grad = 0;
      return 0;
    }
    else if (x < 0)
    {
      double y = x + mu;
      double y2 = y * y;
      grad = y2 * (mu - 2 * x) * mu4_1;
      return 0.5 * y2 * y * (mu - x) * mu4_1;
    }
    else if (x < mu)
    {
      double y = x - mu;
      double y2 = y * y;
      grad = y2 * (mu + 2 * x) * mu4_1;
      return 0.5 * y2 * y * (mu + x) * mu4_1 + 1;
    }
    else
    {
      grad = 0;
      return 1;
    }
  }

  static Eigen::MatrixXd f_DN(const Eigen::Vector3d &x)
  {
    double x_norm_2 = x.squaredNorm();
    return (Eigen::MatrixXd::Identity(3, 3) - x * x.transpose() / x_norm_2) /
           sqrt(x_norm_2);
  }

  template <typename Scalar>
  inline void normalizeFDF(const Eigen::Matrix<Scalar, 3, 1> &x,
                           Eigen::Matrix<Scalar, 3, 1> &xNor,
                           Eigen::Matrix<Scalar, 3, 3> &G) const
  {
    const Scalar a = x(0), b = x(1), c = x(2);
    const Scalar aSqr = a * a, bSqr = b * b, cSqr = c * c;
    const Scalar ab = a * b, bc = b * c, ca = c * a;
    const Scalar xSqrNorm = aSqr + bSqr + cSqr;
    const Scalar xNorm = sqrt(xSqrNorm);
    const Scalar den = xSqrNorm * xNorm;

    xNor = x / xNorm;

    G(0, 0) = bSqr + cSqr;
    G(0, 1) = -ab;
    G(0, 2) = -ca;
    G(1, 0) = -ab;
    G(1, 1) = aSqr + cSqr;
    G(1, 2) = -bc;
    G(2, 0) = -ca;
    G(2, 1) = -bc;
    G(2, 2) = aSqr + bSqr;

    G /= den;
    return;
  }

  inline bool grad_orientation(const Eigen::Vector3d &acc, const double &gAcc,
                               const Eigen::Vector3d &zd, Eigen::Vector3d &grada,
                               double &cost) const
  {
    Eigen::Vector3d zb = acc + Eigen::Vector3d(0, 0, gAcc);
    double zb_norm = zb.norm();
    double violaOri = (zb - zd * zb_norm).squaredNorm() - 1e-4;

    if (violaOri > 0)
    {
      double violaOriPenD = violaOri * violaOri;
      double violaOriPena = violaOriPenD * violaOri;
      cost = violaOriPena;
      Eigen::Vector3d dViodAcc =
          6 * violaOriPenD *
          (Eigen::Matrix3d::Identity() - zd * zb.transpose() / zb_norm).transpose() *
          (zb - zd * zb_norm);
      grada = dViodAcc;
      return true;
    }
    return false;
  }

  template <typename EIGENVEC>
  inline void addTimeIntPenalty(const Mode &mode, const Eigen::VectorXi cons,
                                const Eigen::VectorXi &idxHs,
                                const std::vector<Eigen::MatrixXd> &cfgHs, double &cost,
                                EIGENVEC &gdT, Eigen::MatrixXd &gdC,
                                const std::vector<Eigen::VectorXd> &mInfo,
                                std::vector<int> &idPolys, Config *cfg,
                                const Eigen::MatrixXd &CArm,
                                Eigen::MatrixXd &gdCArm) const
  {
    double pena = 0.0;
    const double vMaxSqr = cfg->velMax * cfg->velMax;
    const double thrAccMinSqr = cfg->thrustAccMin * cfg->thrustAccMin;
    const double thrAccMaxSqr = cfg->thrustAccMax * cfg->thrustAccMax;
    const double bdrMaxSqr = cfg->bodyRateMax * cfg->bodyRateMax;
    Eigen::Vector4d ci = cfg->penaltyPVTB;
    Eigen::Vector4d graspPen = cfg->PenaltyGraspWeight;
    Eigen::Vector3d ellipsoid =
        Eigen::Vector3d(cfg->horizHalfLen, cfg->horizHalfLen, cfg->vertHalfLen);
    Eigen::Vector3d pos, vel, acc, jer, sna;
    Eigen::Vector3d apos, avel, aacc, ajer;
    double step, alpha;
    double s1, s2, s3, s4, s5;
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
    int K;
    Eigen::Matrix3d rotM;
    double signedDist, signedDistSqr, signedDistCub;
    double gradSignedDt;
    Eigen::Vector3d h, zB, czB, xC, yB, xB;
    Eigen::Matrix3d dnczB, dzB, cdzB, dyB, dxB;
    Eigen::Vector3d outerNormal, point;
    Eigen::Vector3d B_outerNormal;
    double eNorm;
    double gradAdc, gradcdT;
    Eigen::Vector3d eNormGd, eNormGd1;
    Eigen::Matrix3d gradSdTxyz;
    Eigen::Vector3d gradSdT;
    double outerNormaldVel;
    Eigen::Matrix<double, 6, 3> gradSdCx, gradSdCy, gradSdCz, gradSdC;
    Eigen::Matrix<double, 6, 3> beta2dOuterNormalTp, beta0dOuterNormalTp;

    double violaVel, violaThrl, violaThrh, violaBdr;
    double violaVelPenaD, violaThrlPenaD, violaThrhPenaD, violaBdrPenaD;
    double violaVelPena, violaThrlPena, violaThrhPena, violaBdrPena;
    Eigen::Matrix<double, 6, 3> gradViolaVc, gradViolaThrlc, gradViolaThrhc,
        gradViolaBdrc;
    double gradViolaVt, gradViolaThrlt, gradViolaThrht, gradViolaBdrt;
    double fThr, sqrMagThr, sqrMagBdr;
    Eigen::Vector3d dfThr, dSqrMagThr, bdr, xyBdr;
    Eigen::Vector3d dSqrMagBdr, rotTrDotJer;
    Eigen::Matrix3d dBdr, dxyBdr;
    Eigen::Vector3d dJerSqrMagBdr;
    Eigen::Matrix3d dJerBdr, dJerxyBdr;
    Eigen::Vector3d w_ee;
    double omg;
    double safeMargin;
    Eigen::Vector3d e3 = Eigen::Vector3d::UnitZ();
    autoDiffParam param;
    Eigen::MatrixXd dCostdc, dCostdEc;
    double dCostdt;
    double mass = 1;
    Eigen::Vector3d Fext = cfg->Fext;

    param.mass = mass;
    param.Fext = Fext;
    param.hasFext = false;
    param.BPosD = Eigen::Vector3d(0.0, 0.0, -cfg->biasArmToBody);
    param.gravAcc = cfg->gravAcc;
    param.EEDownVel = cfg->EEDownVel;
    param.gWeight = cfg->PenaltyGraspWeight;
    bool cons_zone = false;

    std::vector<Eigen::Vector3d> ee_pos_vec;
    for (int i = 0, k = 0; i < mInfo.size(); i++)
    {
      if (mInfo[i](0) == 2.0)
      {
        ee_pos_vec.push_back(mInfo[i].segment(1, 3));
        k++;
      }
    }

    int innerLoop, idx;
    for (int i = 0; i < N; i++)
    {
      const auto &c = b.block<6, 3>(i * 6, 0);
      const auto &E_c = CArm.block<6, 3>(i * 6, 0);
      step = T1(i) / cons(i);
      s1 = 0.0;
      innerLoop = cons(i) + 1;
      for (int j = 0; j < innerLoop; j++)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
        beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
        beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * s1;
        alpha = 1.0 / cons(i) * j;

        /* get pos, vel, acc, jerk, sna for polynomial */
        pos = c.transpose() * beta0;  // c_0->c_5
        vel = c.transpose() * beta1;
        acc = c.transpose() * beta2;
        jer = c.transpose() * beta3;
        sna = c.transpose() * beta4;
        apos = E_c.transpose() * beta0;
        avel = E_c.transpose() * beta1;
        aacc = E_c.transpose() * beta2;
        ajer = E_c.transpose() * beta3;
        if (param.hasFext)
        { /* not used now */
          h = mass * acc - Fext;
        }
        else
        {
          h = mass * acc;
        }

        /* ========== dynamics begin ========== */
        h(2) += mass * param.gravAcc;
        normalizeFDF(h, zB, dzB);
        // Zb
        czB << 0.0, zB(2), -zB(1);
        cdzB << Eigen::RowVector3d::Zero(), dzB.row(2), -dzB.row(1);
        normalizeFDF(czB, yB, dnczB);
        // Yb
        xB = yB.cross(zB);
        dyB = dnczB * cdzB;
        dxB.col(0) = dyB.col(0).cross(zB) + yB.cross(dzB.col(0));
        dxB.col(1) = dyB.col(1).cross(zB) + yB.cross(dzB.col(1));
        dxB.col(2) = dyB.col(2).cross(zB) + yB.cross(dzB.col(2));
        rotM << xB, yB, zB;

        gradSdTxyz.col(0) = dxB * jer;
        gradSdTxyz.col(1) = dyB * jer;
        gradSdTxyz.col(2) = dzB * jer;

        fThr = h.norm();
        dfThr = h / fThr;
        sqrMagThr = fThr * fThr;
        dSqrMagThr = 2 * h;
        rotTrDotJer = rotM.transpose() * jer;
        bdr = rotTrDotJer / fThr;
        xyBdr << -bdr(1), bdr(0), 0.0;
        sqrMagBdr = xyBdr.squaredNorm();
        dBdr = -rotTrDotJer * dfThr.transpose() / (fThr * fThr) -
               rotM.transpose() *
                   (dxB * rotTrDotJer(0) + dyB * rotTrDotJer(1) + dzB * rotTrDotJer(2)) /
                   fThr;
        dxyBdr << -dBdr.row(1), dBdr.row(0), Eigen::RowVector3d::Zero();
        dSqrMagBdr = 2.0 * xyBdr.transpose() * dxyBdr; /* \partial omega / \partial acc */
        dJerBdr = rotM.transpose() / fThr;
        dJerxyBdr << -dJerBdr.row(1), dJerBdr.row(0), Eigen::RowVector3d::Zero();
        dJerSqrMagBdr =
            2.0 * xyBdr.transpose() * dJerxyBdr; /* \partial omega / \partial jerk */

        /* ========== dynamics end ========== */

        violaVel = vel.squaredNorm() - vMaxSqr;
        violaThrl = thrAccMinSqr - sqrMagThr;
        violaThrh = sqrMagThr - thrAccMaxSqr;
        violaBdr = sqrMagBdr - bdrMaxSqr;

        w_ee = rotM * (apos + Eigen::Vector3d(0, 0, -cfg->biasArmToBody)) + pos;

        omg = (j == 0 || j == innerLoop - 1) ? 0.5 : 1.0;

        idx = idxHs(i);
        K = cfgHs[idx].cols();

        /* dilate the approximation */
        Eigen::Vector3d ellipsoid_dilate = ellipsoid;

        /* ========== safety penalty begin ========== */
        ellipsoid_dilate(2) += -e3.transpose() * (E_c.transpose() * beta0) - 0.08;

        {
          double dist2target = dist2GraspPoints(w_ee, mInfo, idPolys);
          safeMargin = cfg->safeMargin * (dist2target > 0.1 ? 1.0 : dist2target / 0.1);
        }
        for (int k = 0; k < K; k++)
        {
          outerNormal = cfgHs[idx].col(k).head<3>();
          B_outerNormal = rotM.transpose() * outerNormal;
          point = cfgHs[idx].col(k).tail<3>();
          beta0dOuterNormalTp = beta0 * outerNormal.transpose();
          gradSdT = gradSdTxyz.transpose() * outerNormal;
          outerNormaldVel = outerNormal.dot(vel);
          beta2dOuterNormalTp = beta2 * outerNormal.transpose();
          gradSdCx = beta2dOuterNormalTp * dxB;  // 6*3
          gradSdCy = beta2dOuterNormalTp * dyB;
          gradSdCz = beta2dOuterNormalTp * dzB;

          eNormGd = B_outerNormal.array() * ellipsoid_dilate.array();
          eNorm = eNormGd.norm();
          eNormGd /= eNorm;
          signedDist = outerNormal.dot(pos - point) + eNorm;
          eNormGd.array() *= ellipsoid_dilate.array();

          signedDist += safeMargin;

          if (signedDist > 0)
          {
            signedDistSqr = signedDist * signedDist;
            signedDistCub = signedDist * signedDistSqr;
            gradSdC = beta0dOuterNormalTp + gradSdCx * eNormGd(0) +
                      gradSdCy * eNormGd(1) + gradSdCz * eNormGd(2);
            gradcdT = -e3.transpose() * (E_c.transpose() * beta1);
            gradAdc = e3.dot(ellipsoid_dilate) * outerNormal.dot(zB) *
                      outerNormal.dot(zB) / eNorm;
            gradSignedDt = alpha * (outerNormaldVel + gradSdT(0) * eNormGd(0) +
                                    gradSdT(1) * eNormGd(1) + gradSdT(2) * eNormGd(2) +
                                    gradAdc * gradcdT);
            gdC.block<6, 3>(i * 6, 0) +=
                omg * step * ci(0) * 3.0 * signedDistSqr * gradSdC;
            gdT(i) +=
                omg * ci(0) *
                (3.0 * signedDistSqr * gradSignedDt * step + signedDistCub / cons(i));
            /* add for varying ellipsoid */
            gdCArm.block<6, 3>(i * 6, 0) += omg * step * ci(0) * 3.0 * signedDistSqr *
                                            gradAdc * -1 * beta0 * e3.transpose();
            /* alpha = j/con(i), step = dt */
            pena += omg * step * ci(0) * signedDistCub;
          }
        } /* ========== safety penalty end ========== */

        /* ========== max velocity penalty begin ========== */
        if (violaVel > 0.0)
        {
          violaVelPenaD = violaVel * violaVel;
          violaVelPena = violaVelPenaD * violaVel;
          violaVelPenaD *= 3.0;
          gradViolaVc = 2.0 * beta1 * vel.transpose();
          gradViolaVt = 2.0 * alpha * vel.transpose() * acc;
          gdC.block<6, 3>(i * 6, 0) += omg * step * ci(1) * violaVelPenaD * gradViolaVc;
          gdT(i) += omg * (ci(1) * violaVelPenaD * gradViolaVt * step +
                           ci(1) * violaVelPena / cons(i));
          pena += omg * step * ci(1) * violaVelPena;
        }
        /* ========== max velocity penalty end ========== */

        /* ========== lowest thrust penalty begin ========== */
        if (violaThrl > 0.0)
        {
          violaThrlPenaD = violaThrl * violaThrl;
          violaThrlPena = violaThrlPenaD * violaThrl;
          violaThrlPenaD *= 3.0;
          gradViolaThrlc = -beta2 * dSqrMagThr.transpose();
          gradViolaThrlt = -alpha * dSqrMagThr.transpose() * jer;
          gdC.block<6, 3>(i * 6, 0) +=
              omg * step * ci(2) * violaThrlPenaD * gradViolaThrlc;
          gdT(i) += omg * (ci(2) * violaThrlPenaD * gradViolaThrlt * step +
                           ci(2) * violaThrlPena / cons(i));
          pena += omg * step * ci(2) * violaThrlPena;
        }
        /* ========== lowest thrust penalty end ========== */

        /* ========== highest thrust penalty begin ========== */
        if (violaThrh > 0.0)
        {
          violaThrhPenaD = violaThrh * violaThrh;
          violaThrhPena = violaThrhPenaD * violaThrh;
          violaThrhPenaD *= 3.0;
          gradViolaThrhc = beta2 * dSqrMagThr.transpose();
          gradViolaThrht = alpha * dSqrMagThr.transpose() * jer;
          gdC.block<6, 3>(i * 6, 0) +=
              omg * step * ci(2) * violaThrhPenaD * gradViolaThrhc;
          gdT(i) += omg * (ci(2) * violaThrhPenaD * gradViolaThrht * step +
                           ci(2) * violaThrhPena / cons(i));
          pena += omg * step * ci(2) * violaThrhPena;
        }
        /* ========== highest thrust penalty end ========== */

        /* ========== max body rate penalty begin ========== */
        if (violaBdr > 0.0)
        {
          violaBdrPenaD = violaBdr * violaBdr;
          violaBdrPena = violaBdrPenaD * violaBdr;
          violaBdrPenaD *= 3.0;
          gradViolaBdrc =
              beta2 * dSqrMagBdr.transpose() + beta3 * dJerSqrMagBdr.transpose();
          gradViolaBdrt = alpha * (dSqrMagBdr.dot(jer) + dJerSqrMagBdr.dot(sna));
          gdC.block<6, 3>(i * 6, 0) += omg * step * ci(3) * violaBdrPenaD * gradViolaBdrc;
          gdT(i) += omg * (ci(3) * violaBdrPenaD * gradViolaBdrt * step +
                           ci(3) * violaBdrPena / cons(i));
          pena += omg * step * ci(3) * violaBdrPena;
        }
        /* ========== max body rate penalty end ========== */

        /* ========== orientation penalty begin ========== */
        double orientation_cost;
        Eigen::Vector3d pCost_pAcc;
        if (needOrientationPenalty(i - 1, mInfo, idPolys) && j == 0)
        {
          Eigen::Vector3d zd;
          int type = mInfo[getInfoId(i - 1, idPolys)](0);
          if (type == 1.0 || type == 2.0)
            zd = mInfo[getInfoId(i - 1, idPolys)].segment(4, 3).normalized();
          if (type == 3.0)
            zd = mInfo[getInfoId(i - 1, idPolys)].segment(7, 3).normalized();
          if (grad_orientation(acc, cfg->gravAcc, zd, pCost_pAcc, orientation_cost))
          {
            gdC.block<6, 3>(i * 6, 0) +=
                omg * graspPen(3) * step * beta2 * pCost_pAcc.transpose();
            gdT(i) += omg * graspPen(3) *
                      (step * alpha * (pCost_pAcc.dot(jer)) + orientation_cost / cons(i));
            pena += omg * step * graspPen(3) * orientation_cost;
          }
        }
        /* ========== orientation penalty end ========== */

        /* ========== grasp position penalty begin ========== */
        double position_cost = 0.0;
        Eigen::Vector3d ee_pos;
        if (needPositionPenalty(i - 1, mInfo, idPolys) && j == 0)
        {
          if (SpecificPointType(i - 1, mInfo, idPolys) == 2.0)
          {
            ee_pos = mInfo[getInfoId(i - 1, idPolys)].segment(1, 3);
            param.xyzConsVelVals = mInfo[getInfoId(i - 1, idPolys)].segment(8, 3);
            param.xyzVelIsCons = mInfo[getInfoId(i - 1, idPolys)].segment(11, 3);

            if (mInfo[getInfoId(i - 1, idPolys)].segment(14, 3).norm() != 0.0)
            {
              /* cons_zone is used to begin a segment of any axis constraint */
              param.xyzConsVals = ee_pos;
              param.xyzIsCons = mInfo[getInfoId(i - 1, idPolys)].segment(14, 3);
              if (!cons_zone) cons_zone = true;
              if (cons_zone)
              {
                if (mInfo.size() <= getInfoId(i - 1, idPolys) + 1 ||
                    mInfo[getInfoId(i - 1, idPolys) + 1](0) != 2.0)
                {
                  cons_zone = false;
                }
              }
            }
          }
          else if (SpecificPointType(i - 1, mInfo, idPolys) == 3.0)
          {
            ee_pos = w_ee;
            param.xyzVelIsCons = mInfo[getInfoId(i - 1, idPolys)].segment(14, 3);
            param.xyzConsVelVals = mInfo[getInfoId(i - 1, idPolys)].segment(11, 3);
            if (mInfo[getInfoId(i - 1, idPolys)].segment(17, 3).norm() != 0.0)
            {
              param.xyzIsCons = mInfo[getInfoId(i - 1, idPolys)].segment(17, 3);
              param.xyzConsVals = ee_pos;
              if (!cons_zone) cons_zone = true;
              if (cons_zone)
              {
                if (mInfo.size() <= getInfoId(i - 1, idPolys) + 1 ||
                    mInfo[getInfoId(i - 1, idPolys) + 1](0) != 3.0)
                {
                  cons_zone = false;
                }
              }
            }
          }
          param.objPos = ee_pos;
          if (grad_position(c, E_c, s1, param, false, dCostdc, dCostdEc, dCostdt,
                            position_cost))
          {
            gdC.block<6, 3>(i * 6, 0) += omg * step * dCostdc;
            gdCArm.block<6, 3>(i * 6, 0) += omg * step * dCostdEc;
            gdT(i) += omg * (step * alpha * dCostdt + position_cost / cons(i));
            pena += omg * step * position_cost;
          }
        }

        /* ========== xyz constraint begin ========== */
        double xyz_cost = 0.0;
        if (cons_zone)
        {
          if (grad_position(c, E_c, s1, param, cons_zone, dCostdc, dCostdEc, dCostdt,
                            xyz_cost))
          {
            gdC.block<6, 3>(i * 6, 0) += omg * step * dCostdc;
            gdCArm.block<6, 3>(i * 6, 0) += omg * step * dCostdEc;
            gdT(i) += omg * (step * alpha * dCostdt + xyz_cost / cons(i));
            pena += omg * step * xyz_cost;
          }
        }
        /* ========== xyz constraint end ========== */

        s1 += step;
      }
    }
    cost += pena;
    return;
  }

  bool grad_position(const Eigen::MatrixXd &c, const Eigen::MatrixXd &E_c,
                     const double &t, const autoDiffParam &param, const bool &cons_zone,
                     Eigen::MatrixXd &dCostdc, Eigen::MatrixXd &dCostdEc, double &dCostdt,
                     double &ret_cost) const
  {
    autodiff::ArrayXreal all_params(37);
    autodiff::real cost;
    Eigen::VectorXd full_gradient;
    autodiff::MatrixXreal c_real, E_c_real;
    autodiff::real t_real;

    c_real = c.cast<autodiff::real>();
    E_c_real = E_c.cast<autodiff::real>();
    t_real = static_cast<autodiff::real>(t);

    all_params.head(18) = Eigen::Map<const autodiff::VectorXreal>(c_real.data(), 18);
    all_params.segment(18, 18) =
        Eigen::Map<const autodiff::VectorXreal>(E_c_real.data(), 18);
    all_params[36] = t_real;

    auto cost_function = [&](const autodiff::ArrayXreal &params) -> autodiff::real
    {
      autodiff::MatrixXreal c_param =
          Eigen::Map<const autodiff::MatrixXreal>(params.data(), 6, 3);
      autodiff::MatrixXreal E_c_param =
          Eigen::Map<const autodiff::MatrixXreal>(params.data() + 18, 6, 3);
      autodiff::real t_param = params[36];

      /* if cons_zone is true, then the xyz constraint is used */
      if (cons_zone)
      {
        return xyz_constraint(t_param, c_param, E_c_param, param);
      }
      else
      {
        return reach_position(t_param, c_param, E_c_param, param);
      }
    };

    full_gradient = autodiff::gradient(cost_function, autodiff::wrt(all_params),
                                       autodiff::at(all_params), cost);
    dCostdc = Eigen::Map<const Eigen::MatrixXd>(full_gradient.data(), 6, 3);
    dCostdEc = Eigen::Map<const Eigen::MatrixXd>(full_gradient.data() + 18, 6, 3);
    dCostdt = full_gradient[36];
    ret_cost = static_cast<double>(cost);
    return true;
  }

  autodiff::real reach_position(const autodiff::real &t, const autodiff::MatrixXreal &c,
                                const autodiff::MatrixXreal &E_c,
                                const autoDiffParam &param) const
  {
    autodiff::real s1, s2, s3, s4, s5;
    autodiff::VectorXreal beta0(6), beta1(6), beta2(6), beta3(6), beta4(6);
    autodiff::Vector3real pos, vel, acc, jer, sna, apos, avel, aacc;
    autodiff::Vector3real h, zB, czB, yB, xB;
    autodiff::Matrix3real dnczB, dzB, cdzB, dyB, dxB;
    autodiff::Matrix3real rotM;
    autodiff::Vector3real W_pos_e;
    autodiff::real cost_pos = 0.0, cost_vel = 0.0;
    autodiff::Vector3real xyBdr, rotTrDotJer, bdr, W_bdr, W_vel_e, W_vel_des_e;
    autodiff::Matrix3real W_bdr_inv;
    autodiff::real fThr, sqrMagThr;
    s1 = t;
    s2 = s1 * s1;
    s3 = s2 * s1;
    s4 = s2 * s2;
    s5 = s4 * s1;
    beta0 << 1.0, s1, s2, s3, s4, s5;
    beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
    beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
    beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
    beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * s1;
    pos = c.transpose() * beta0;
    vel = c.transpose() * beta1;
    acc = c.transpose() * beta2;
    jer = c.transpose() * beta3;
    sna = c.transpose() * beta4;
    apos = E_c.transpose() * beta0;
    avel = E_c.transpose() * beta1;
    aacc = E_c.transpose() * beta2;

    if (param.hasFext)
      h = param.mass * acc - param.Fext;
    else
      h = param.mass * acc;
    h(2) += param.gravAcc;

    normalizeFDF(h, zB, dzB);
    czB << 0.0, zB(2), -zB(1);
    normalizeFDF(czB, yB, dnczB);
    xB = yB.cross(zB);
    rotM << xB, yB, zB;

    fThr = h.norm();
    sqrMagThr = fThr * fThr;
    rotTrDotJer = rotM.transpose() * jer;
    bdr = rotTrDotJer / fThr;
    xyBdr << -bdr(1), bdr(0), 0.0;

    /* grasp position */
    W_pos_e = rotM * (apos + param.BPosD) + pos;
    cost_pos = (W_pos_e - param.objPos).norm();
    cost_pos = cost_pos * cost_pos * cost_pos;

    /* grasp velocity */
    W_bdr = rotM * xyBdr;
    W_bdr_inv << 0, -W_bdr(2), W_bdr(1), W_bdr(2), 0, -W_bdr(0), -W_bdr(1), W_bdr(0), 0;
    W_vel_e = vel + W_bdr_inv * rotM * (apos + param.BPosD) + rotM * avel;

    W_vel_des_e = rotM * param.xyzConsVelVals;
    for (int i = 0; i < 3; i++)
    {
      if (param.xyzVelIsCons(i) == 1)
      {
        cost_vel += (W_vel_e(i) - W_vel_des_e(i)) * (W_vel_e(i) - W_vel_des_e(i));
      }
      if (param.xyzVelIsCons(i) == 2)
      {
        if (W_vel_e(i) < W_vel_des_e(i))
        {
          cost_vel += (W_vel_e(i) - W_vel_des_e(i)) * (W_vel_e(i) - W_vel_des_e(i));
        }
      }
      if (param.xyzVelIsCons(i) == 3)
      {
        if (W_vel_e(i) > W_vel_des_e(i))
        {
          cost_vel += (W_vel_e(i) - W_vel_des_e(i)) * (W_vel_e(i) - W_vel_des_e(i));
        }
      }
    }

    /* start grasp */
    // cost_vel = (W_vel_e - W_vel_des_e).norm();
    // cost_vel = cost_vel * cost_vel * cost_vel;
    /* end grasp */

    /* start strike*/
    // autodiff::Vector3real e1;
    // e1 << 1.0, 0.0, 0.0;
    // if (W_vel_e.dot(e1) < 0.1)
    // {
    //   cost_vel = W_vel_e.dot(e1) - 0.1;
    // }
    // else
    // {
    //   cost_vel = 0.0;
    // }
    /* end strike*/

    return cost_pos * param.gWeight(0) + cost_vel * param.gWeight(1);
  }

  autodiff::real xyz_constraint(const autodiff::real &t, const autodiff::MatrixXreal &c,
                                const autodiff::MatrixXreal &E_c,
                                const autoDiffParam &param) const
  {
    autodiff::real s1, s2, s3, s4, s5;
    autodiff::VectorXreal beta0(6), beta1(6), beta2(6), beta3(6), beta4(6);
    autodiff::Vector3real pos, vel, acc, jer, sna, apos, avel, aacc;
    autodiff::Vector3real h, zB, czB, yB, xB;
    autodiff::Matrix3real dnczB, dzB, cdzB, dyB, dxB;
    autodiff::Matrix3real rotM;
    autodiff::Vector3real W_pos_e;
    autodiff::real cost_pos, cost_vel;
    autodiff::Vector3real xyBdr, rotTrDotJer, bdr, W_bdr, W_vel_e, W_vel_des_e;
    autodiff::Matrix3real W_bdr_inv;
    autodiff::real fThr, sqrMagThr;
    autodiff::Vector3real xyz_cons;
    s1 = t;
    s2 = s1 * s1;
    s3 = s2 * s1;
    s4 = s2 * s2;
    s5 = s4 * s1;
    beta0 << 1.0, s1, s2, s3, s4, s5;
    beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
    beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
    beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
    beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * s1;
    pos = c.transpose() * beta0;
    vel = c.transpose() * beta1;
    acc = c.transpose() * beta2;
    jer = c.transpose() * beta3;
    sna = c.transpose() * beta4;
    apos = E_c.transpose() * beta0;
    avel = E_c.transpose() * beta1;
    aacc = E_c.transpose() * beta2;

    h = acc;
    h(2) += param.gravAcc;
    normalizeFDF(h, zB, dzB);
    czB << 0.0, zB(2), -zB(1);
    normalizeFDF(czB, yB, dnczB);
    xB = yB.cross(zB);
    rotM << xB, yB, zB;

    fThr = h.norm();
    sqrMagThr = fThr * fThr;
    rotTrDotJer = rotM.transpose() * jer;
    bdr = rotTrDotJer / fThr;
    xyBdr << -bdr(1), bdr(0), 0.0;

    /* pull position */
    W_pos_e = rotM * (apos + param.BPosD) + pos;
    xyz_cons = param.xyzIsCons.cwiseProduct(param.xyzConsVals - W_pos_e);
    cost_pos = xyz_cons.norm();

    /* pull veolcity */
    W_bdr = rotM * xyBdr;
    W_bdr_inv << 0, -W_bdr(2), W_bdr(1), W_bdr(2), 0, -W_bdr(0), -W_bdr(1), W_bdr(0), 0;
    W_vel_e = vel + W_bdr_inv * rotM * (apos + param.BPosD) + rotM * avel;
    W_vel_des_e = param.xyzIsCons.cwiseProduct(W_vel_e);
    cost_vel = W_vel_des_e.norm();

    return cost_pos * param.gWeight(2) + cost_vel * param.gWeight(2);
  }

 public:
  inline void reset(const Eigen::Matrix3d &headState, const Eigen::Matrix3d &tailState,
                    const int &pieceNum)
  {
    N = pieceNum;
    headPVA = headState;
    tailPVA = tailState;
    T1.resize(N);
    A.create(6 * N, 6, 6);
    b.resize(6 * N, 3);
    gdC.resize(6 * N, 3);
    return;
  }

  inline void setGrasp(const int &pt_id,
                       const Eigen::MatrixXd &graspState = Eigen::MatrixXd::Zero(3, 3))
  {
    graspPVA = graspState;
    grasp_pt = pt_id;
    return;
  }

  inline void generate(const Eigen::MatrixXd &inPs, const Eigen::VectorXd &ts)
  {
    T1 = ts;
    T2 = T1.cwiseProduct(T1);
    T3 = T2.cwiseProduct(T1);
    T4 = T2.cwiseProduct(T2);
    T5 = T4.cwiseProduct(T1);

    A.reset();
    b.setZero();
    A(0, 0) = 1.0;
    A(1, 1) = 1.0;
    A(2, 2) = 2.0;
    b.row(0) = headPVA.col(0).transpose();
    b.row(1) = headPVA.col(1).transpose();
    b.row(2) = headPVA.col(2).transpose();
    for (int i = 0; i < N - 1; i++)
    {
      /* jerk */
      A(6 * i + 3, 6 * i + 3) = 6.0;
      A(6 * i + 3, 6 * i + 4) = 24.0 * T1(i);
      A(6 * i + 3, 6 * i + 5) = 60.0 * T2(i);
      A(6 * i + 3, 6 * i + 9) = -6.0;

      A(6 * i + 4, 6 * i + 4) = 24.0;
      A(6 * i + 4, 6 * i + 5) = 120.0 * T1(i);
      A(6 * i + 4, 6 * i + 10) = -24.0;

      /* fixed position */
      A(6 * i + 5, 6 * i) = 1.0;
      A(6 * i + 5, 6 * i + 1) = T1(i);
      A(6 * i + 5, 6 * i + 2) = T2(i);
      A(6 * i + 5, 6 * i + 3) = T3(i);
      A(6 * i + 5, 6 * i + 4) = T4(i);
      A(6 * i + 5, 6 * i + 5) = T5(i);

      /* pos */
      A(6 * i + 6, 6 * i) = 1.0;
      A(6 * i + 6, 6 * i + 1) = T1(i);
      A(6 * i + 6, 6 * i + 2) = T2(i);
      A(6 * i + 6, 6 * i + 3) = T3(i);
      A(6 * i + 6, 6 * i + 4) = T4(i);
      A(6 * i + 6, 6 * i + 5) = T5(i);
      A(6 * i + 6, 6 * i + 6) = -1.0;

      /* vel */
      A(6 * i + 7, 6 * i + 1) = 1.0;
      A(6 * i + 7, 6 * i + 2) = 2 * T1(i);
      A(6 * i + 7, 6 * i + 3) = 3 * T2(i);
      A(6 * i + 7, 6 * i + 4) = 4 * T3(i);
      A(6 * i + 7, 6 * i + 5) = 5 * T4(i);
      A(6 * i + 7, 6 * i + 7) = -1.0;

      /* acc */
      A(6 * i + 8, 6 * i + 2) = 2.0;
      A(6 * i + 8, 6 * i + 3) = 6 * T1(i);
      A(6 * i + 8, 6 * i + 4) = 12 * T2(i);
      A(6 * i + 8, 6 * i + 5) = 20 * T3(i);
      A(6 * i + 8, 6 * i + 8) = -2.0;

      b.row(6 * i + 5) = inPs.col(i).transpose();
    }

    A(6 * N - 3, 6 * N - 6) = 1.0;
    A(6 * N - 3, 6 * N - 5) = T1(N - 1);
    A(6 * N - 3, 6 * N - 4) = T2(N - 1);
    A(6 * N - 3, 6 * N - 3) = T3(N - 1);
    A(6 * N - 3, 6 * N - 2) = T4(N - 1);
    A(6 * N - 3, 6 * N - 1) = T5(N - 1);
    A(6 * N - 2, 6 * N - 5) = 1.0;
    A(6 * N - 2, 6 * N - 4) = 2 * T1(N - 1);
    A(6 * N - 2, 6 * N - 3) = 3 * T2(N - 1);
    A(6 * N - 2, 6 * N - 2) = 4 * T3(N - 1);
    A(6 * N - 2, 6 * N - 1) = 5 * T4(N - 1);
    A(6 * N - 1, 6 * N - 4) = 2;
    A(6 * N - 1, 6 * N - 3) = 6 * T1(N - 1);
    A(6 * N - 1, 6 * N - 2) = 12 * T2(N - 1);
    A(6 * N - 1, 6 * N - 1) = 20 * T3(N - 1);

    b.row(6 * N - 3) = tailPVA.col(0).transpose();
    b.row(6 * N - 2) = tailPVA.col(1).transpose();
    b.row(6 * N - 1) = tailPVA.col(2).transpose();
    // Avoid printing BandedSystem directly since it lacks ostream operator<<

    A.factorizeLU();
    A.solve(b);
    return;
  }

  inline double getTrajJerkCost() const
  {
    double objective = 0.0;
    for (int i = 0; i < N; i++)
    {
      objective += 36.0 * b.row(6 * i + 3).squaredNorm() * T1(i) +
                   144.0 * b.row(6 * i + 4).dot(b.row(6 * i + 3)) * T2(i) +
                   192.0 * b.row(6 * i + 4).squaredNorm() * T3(i) +
                   240.0 * b.row(6 * i + 5).dot(b.row(6 * i + 3)) * T3(i) +
                   720.0 * b.row(6 * i + 5).dot(b.row(6 * i + 4)) * T4(i) +
                   720.0 * b.row(6 * i + 5).squaredNorm() * T5(i);
    }
    return objective;
  }

  inline int SpecificPointType(const int &id, const std::vector<Eigen::VectorXd> &mInfo,
                               const std::vector<int> &idPolys) const
  {
    for (int i = 0; i < idPolys.size(); i++)
    {
      /* if active polyhedron generation */
      if (id == idPolys[i]) return mInfo[i](0);
    }
    return -1;
  }
  inline int getInfoId(const int &id, const std::vector<int> &idPolys) const
  {
    for (int i = 0; i < idPolys.size(); i++)
    {
      if (id == idPolys[i]) return i;
    }
    std::cout << "\033[31m[MINCO_BASE] Point id not found: " << id << "\033[0m"
              << std::endl;
    return -1;
  }

  inline bool needOrientationPenalty(const int &id,
                                     const std::vector<Eigen::VectorXd> &mInfo,
                                     const std::vector<int> &idPolys) const
  {
    if (SpecificPointType(id, mInfo, idPolys) == 1.0 ||
        SpecificPointType(id, mInfo, idPolys) == 2.0)
    {
      if (mInfo[getInfoId(id, idPolys)][7] != 0.0) return true;
    }
    if (SpecificPointType(id, mInfo, idPolys) == 3.0)
    {
      if (mInfo[getInfoId(id, idPolys)][10] != 0.0) return true;
    }
    return false;
  }
  inline bool needPositionPenalty(const int &id,
                                  const std::vector<Eigen::VectorXd> &mInfo,
                                  const std::vector<int> &idPolys) const
  {
    if (SpecificPointType(id, mInfo, idPolys) == 2.0 ||
        SpecificPointType(id, mInfo, idPolys) == 3.0)
      return true;
    return false;
  }
  inline double dist2GraspPoints(const Eigen::Vector3d &pos,
                                 const std::vector<Eigen::VectorXd> &mInfo,
                                 const std::vector<int> &idPolys) const
  {
    double min_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < mInfo.size(); i++)
    {
      if (mInfo[i](0) == 2.0)
        min_dist = std::min(min_dist, (pos - mInfo[i].segment(1, 3)).norm());
    }
    return min_dist;
  }
  template <typename EIGENVEC, typename EIGENMAT>
  inline void evalBaseTrajCostGrad(const Mode &mode, const Eigen::VectorXi &cons,
                                   const Eigen::VectorXi &idxHs,
                                   const std::vector<Eigen::MatrixXd> &cfgHs,
                                   double &cost, EIGENVEC &gdT, EIGENMAT &gdInPs,
                                   const std::vector<Eigen::VectorXd> &mInfo,
                                   std::vector<int> &idPolys, Config *cfg,
                                   const Eigen::MatrixXd &CArm, Eigen::MatrixXd &gdCArm)
  {
    gdT.setZero();
    gdInPs.setZero();
    gdC.setZero();
    gdCArm.setZero();

    /* get jerk cost */
    cost += getTrajJerkCost();
    /*
      J is the jerk cost
      ∂J / ∂T and ∂J / ∂C
    */
    addGradJbyT(gdT);
    addGradJbyC(gdC);

    /*
      G is the constraints
      ∂G / ∂T and ∂G / ∂C
    */

    /* all the constraints penalty is added here */
    addTimeIntPenalty(mode, cons, idxHs, cfgHs, cost, gdT, gdC, mInfo, idPolys, cfg, CArm,
                      gdCArm);

    /*
      C is the polynomial coefficient
      ∂C / ∂T and ∂C / ∂P
    */
    solveAdjGradC(gdC);
    addPropCtoT(gdC, gdT);
    addPropCtoP(gdC, gdInPs);

    /*
      the gradient of the grasp point must be zero
    */
  }

  inline Trajectory<5> getTraj(void) const
  {
    Trajectory<5> traj;
    traj.reserve(N);
    for (int i = 0; i < N; i++)
    {
      traj.emplace_back(T1(i), b.block<6, 3>(6 * i, 0).transpose().rowwise().reverse());
    }
    return traj;
  }
};
#endif