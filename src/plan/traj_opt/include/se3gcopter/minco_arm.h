#ifndef MINCO_ARM_H
#define MINCO_ARM_H
#include "neural_network/workspace_model.h"
#include "traj_opt/config.h"
#include "trajectory.h"

class MINCO_S3_ARM
{
 public:
  MINCO_S3_ARM() = default;
  ~MINCO_S3_ARM() { A.destroy(); }

 private:
  int N;
  Eigen::Matrix3d headPVA;
  Eigen::Matrix3d tailPVA;
  Eigen::Matrix3d graspPVA;
  BandedSystem A;
  Eigen::MatrixX3d b;

  // Temp variables
  Eigen::VectorXd T1;
  Eigen::VectorXd T2;
  Eigen::VectorXd T3;
  Eigen::VectorXd T4;
  Eigen::VectorXd T5;
  Eigen::MatrixXd gdC;

  ros::NodeHandle nh_;

 private:
  template <typename EIGENVEC>
  inline void addGradJbyT(EIGENVEC &gdT) const
  {
    for (int i = 0; i < N; i++)
    {
      gdT(i) += (36.0 * b.row(6 * i + 3).squaredNorm() +
                 288.0 * b.row(6 * i + 4).dot(b.row(6 * i + 3)) * T1(i) +
                 576.0 * b.row(6 * i + 4).squaredNorm() * T2(i) +
                 720.0 * b.row(6 * i + 5).dot(b.row(6 * i + 3)) * T2(i) +
                 2880.0 * b.row(6 * i + 5).dot(b.row(6 * i + 4)) * T3(i) +
                 3600.0 * b.row(6 * i + 5).squaredNorm() * T4(i));
    }
    return;
  }

  template <typename EIGENMAT>
  inline void addGradJbyC(EIGENMAT &gdC) const
  {
    for (int i = 0; i < N; i++)
    {
      gdC.row(6 * i + 5) +=
          1 * (240.0 * b.row(6 * i + 3) * T3(i) + 720.0 * b.row(6 * i + 4) * T4(i) +
               1440.0 * b.row(6 * i + 5) * T5(i));
      gdC.row(6 * i + 4) +=
          1 * (144.0 * b.row(6 * i + 3) * T2(i) + 384.0 * b.row(6 * i + 4) * T3(i) +
               720.0 * b.row(6 * i + 5) * T4(i));
      gdC.row(6 * i + 3) +=
          1 * (72.0 * b.row(6 * i + 3) * T1(i) + 144.0 * b.row(6 * i + 4) * T2(i) +
               240.0 * b.row(6 * i + 5) * T3(i));
    }
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

  inline void solveAdjGradC(Eigen::MatrixXd &gdC) const
  {
    A.solveAdj(gdC);
    return;
  }

  template <typename EIGENMAT>
  inline void addPropCtoP(const Eigen::MatrixXd &adjGdC, EIGENMAT &gdInP) const
  {
    for (int i = 0; i < N - 1; i++)
    {
      gdInP.col(i)(0) += adjGdC.row(6 * i + 5)(0);
      gdInP.col(i)(1) += adjGdC.row(6 * i + 5)(1);
      gdInP.col(i)(2) += adjGdC.row(6 * i + 5)(2);
    }
    return;
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

      /* crackle */
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

    A.factorizeLU();
    A.solve(b);

    return;
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

  inline const Eigen::MatrixX3d &getCoeffs(void) const { return b; }

  template <typename EIGENVEC>
  inline void addTimeIntPenalty(const Mode &mode, const Eigen::VectorXi &cons,
                                WorkspaceProbabilityModel &workspace_model, double &cost,
                                Eigen::MatrixXd &gdC, EIGENVEC &gdT, Config *cfg,
                                const std::vector<Eigen::VectorXd> &mInfo,
                                std::vector<int> &idPolys)
  {
    double pena = 0.0;
    const double armVelMax = cfg->armVelMax;
    const double armVelMaxSqr = cfg->armVelMax * cfg->armVelMax;
    Eigen::Vector3d lam = cfg->penaltyArm;
    lam = lam.cwiseProduct(lam);

    double s1, s2, s3, s4, s5;
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2;
    double step;
    Eigen::Vector3d pos, vel, acc;
    double alpha;
    double violaVel, violaVelPenaD, violaVelPena;
    double gradViolaVt;
    double violaPos, violaPosPenaD, violaPosPena;
    double grad_nn1, grad_nn2, grad_nn3, prob;

    double omg;
    Eigen::Vector3d e3(0, 0, 1);
    Eigen::Matrix<double, 6, 3> gradViolaVc;

    std::vector<double> input_vec;
    input_vec.clear();

    int innerLoop;

    /* calculate the penalty of workspace */
    /*
        @misc{chen2025aerialgraspingmaximizingdeltaarm,
              title={Aerial Grasping via Maximizing Delta-Arm Workspace Utilization},
              author={Haoran Chen and Weiliang Deng and Biyu Ye and Yifan Xiong and
       Zongliang Pan and Ximin Lyu}, year={2025}, eprint={2506.15539},
              archivePrefix={arXiv},
              primaryClass={cs.RO},
              url={https://arxiv.org/abs/2506.15539},
        }
    */
    /* make full use of the parallel computing of nn */
    /* so we concatenate the pos to a matrix first */
    for (int i = 0; i < N; i++)
    {
      const auto &c = b.block<6, 3>(i * 6, 0);
      s1 = 0.0;
      innerLoop = cons(i) + 1;
      for (int j = 0; j < innerLoop; j++) /* integral */
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s3 * s1;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        step = T1(i) / cons(i);
        pos = c.transpose() * beta0;
        input_vec.push_back(pos(0));
        input_vec.push_back(pos(1));
        input_vec.push_back(pos(2));
        s1 += step;
      }
    }
    Eigen::VectorXd input_vec_eigen =
        Eigen::Map<Eigen::VectorXd>(input_vec.data(), input_vec.size());
    auto [prediction, grad_vec] = workspace_model.forward(input_vec_eigen, true);
    /* output the prediction and gradient of workspace */
    const std::vector<double> predictions(prediction.data(),
                                          prediction.data() + prediction.size());
    const std::vector<double> gradients(grad_vec.data(),
                                        grad_vec.data() + grad_vec.size());

    int count = -1;

    /* add the all the soft penalty */
    for (int i = 0; i < N; i++)
    {
      const auto &c = b.block<6, 3>(i * 6, 0);
      step = T1(i) / cons(i);
      s1 = 0.0;
      innerLoop = cons(i) + 1;
      for (int j = 0; j < innerLoop; j++) /* integral */
      {
        count++;
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s3 * s1;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
        alpha = 1.0 / cons(i) * j;
        pos = c.transpose() * beta0;
        vel = c.transpose() * beta1;
        acc = c.transpose() * beta2;
        omg = (j == 0 || j == innerLoop - 1) ? 0.5 : 1.0;

        /* ========== workspace cost begin ========== */
        prob = predictions[count];
        if (prob > 1 || prob < 0 || std::isnan(prob))
        {
          prob = 1;
        }
        grad_nn1 = gradients[count * 3];
        grad_nn2 = gradients[count * 3 + 1];
        grad_nn3 = gradients[count * 3 + 2];
        Eigen::Vector3d grad_nn = Eigen::Vector3d(grad_nn1, grad_nn2, grad_nn3);

        violaPos = 0.5 - prob;
        if (violaPos > 0.0)
        {
          violaPosPenaD = violaPos * violaPos;
          violaPosPena = violaPosPenaD * violaPos;
          gdC.block<6, 3>(i * 6, 0) +=
              -omg * step * lam(0) * 3 * violaPosPenaD * beta0 * grad_nn.transpose();
          gdT(i) += omg * lam(0) *
                    (-step * 3 * alpha * violaPosPenaD * grad_nn.dot(vel) +
                     violaPosPena / cons(i));
          pena += omg * step * lam(0) * violaPosPena;
        }
        /* ========== workspace cost end ========== */

        /* ========== extend cost begin ========== */
        /* some time we expect the ee can extend as much as possible */
        if (isGraspPoint(i - 1, mInfo, idPolys) && j == 0)
        {
          double posUp = 0.25;
          double violaPosUp = posUp * posUp - pos.dot(e3) * pos.dot(e3);
          if (true)
          {
            double violaPosUpPenaD = violaPosUp * violaPosUp;
            double violaPosUpPena = violaPosUpPenaD * violaPosUp;
            Eigen::Vector3d gradPenaltyPos = -6 * violaPosUpPenaD * pos.dot(e3) * e3;
            gdC.block<6, 3>(i * 6, 0) +=
                omg * step * lam(2) * beta0 * gradPenaltyPos.transpose();
            double PpenPt = alpha * step * gradPenaltyPos.dot(vel);
            gdT(i) += omg * lam(2) * (PpenPt + violaPosUpPena / cons(i));
            pena += omg * step * lam(2) * violaPosUpPena;
          }
        }
        /* ========== extend cost end ========== */

        /* ========== velocity cost begin ========== */
        violaVel = vel.squaredNorm() - armVelMaxSqr;
        if (violaVel > 0.0)
        {
          violaVelPenaD = violaVel * violaVel;
          violaVelPena = violaVelPenaD * violaVel;
          violaVelPenaD *= 3.0;
          gradViolaVc = 2.0 * beta1 * vel.transpose();
          gradViolaVt = 2.0 * alpha * vel.transpose() * acc;
          gdC.block<6, 3>(i * 6, 0) += omg * step * lam(1) * violaVelPenaD * gradViolaVc;
          gdT(i) += omg * lam(1) *
                    (violaVelPenaD * gradViolaVt * step + violaVelPena / cons(i));
          pena += omg * step * lam(1) * violaVelPena;
        }
        /* ========== velocity cost end ========== */

        s1 += step;
      }
    }

    cost += pena;
    return;
  }

  inline bool isGraspPoint(const int &id, const std::vector<Eigen::VectorXd> &mInfo,
                           const std::vector<int> &idPolys)
  {
    for (int i = 0; i < idPolys.size(); i++)
    {
      if (id == idPolys[i] && mInfo[i](0) == 2.0) return true;
    }
    return false;
  }
  template <typename EIGENVEC, typename EIGENMAT>
  inline void evalArmTrajCostGrad(const Mode &mode, const Eigen::VectorXi &cons,
                                  WorkspaceProbabilityModel &workspace_model,
                                  EIGENMAT &gdPs, EIGENVEC &gdT, double &arm_cost,
                                  Config *cfg, const std::vector<Eigen::VectorXd> &mInfo,
                                  std::vector<int> &idPolys,
                                  const Eigen::MatrixXd &gdC_ARM)
  {
    gdC.setZero();
    gdT.setZero();
    gdPs.setZero();

    /* get jerk cost */
    arm_cost += getTrajJerkCost();
    if (std::isnan(arm_cost))
    {
      arm_cost = 0.0;
      /* if the jerk cost is nan, then the trajectory is not valid */
      /* sometime will occur */
      ROS_ERROR("Arm Jerk cost is nan");
    }

    // ∂J / ∂T and ∂J / ∂C
    addGradJbyT(gdT);
    addGradJbyC(gdC);
    gdC += gdC_ARM;

    /* the most import penalty is added here */
    addTimeIntPenalty(mode, cons, workspace_model, arm_cost, gdC, gdT, cfg, mInfo,
                      idPolys);

    //  ∂C / ∂T and ∂C / ∂P
    solveAdjGradC(gdC);
    addPropCtoT(gdC, gdT);
    addPropCtoP(gdC, gdPs);
  }
};

#endif