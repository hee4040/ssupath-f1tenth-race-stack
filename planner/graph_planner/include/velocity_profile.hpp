#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <vector>
#include "offline_params.hpp"


using Eigen::VectorXd;
using Eigen::MatrixXd;


class VpForwardBackward {
public:
   OfflineParams params;


   VpForwardBackward() = default;
   explicit VpForwardBackward(const OfflineParams& p) : params(p) {}


   void setParams(const OfflineParams& p) { params = p; }


   // ============================================================
   //  Public Interface
   // ============================================================
   VectorXd calcVelProfile(const VectorXd& kappa,
                           const VectorXd& el_lengths,
                           double v_start,
                           double v_end)
   {
       int N = kappa.size();
       if (el_lengths.size() != (params.closed_track ? N : N - 1))
           throw std::runtime_error("el_lengths size mismatch in calcVelProfile()");


       // --- 곡률 기반 반지름 계산
       VectorXd radii(N);
       for (int i = 0; i < N; ++i)
           radii(i) = (std::abs(kappa(i)) < 1e-9) ? 1e9 : 1.0 / std::abs(kappa(i));


       // --- 로컬 GGV 구성
       MatrixXd loc_gg(N, 2);
       loc_gg.col(0).setConstant(params.ax_max);
       loc_gg.col(1).setConstant(params.ay_max);


       // --- 모터 한계 (v, ax_max_machines)
       MatrixXd ax_max_machines(3, 2);
       ax_max_machines << 0.0, params.ax_max,
                          10.0, params.ax_max * 0.8,
                          20.0, params.ax_max * 0.6;


       // --- 마찰계수
       VectorXd mu = VectorXd::Constant(N, params.mu);
       double mu_mean = mu.mean();


       // --- p_ggv: [vx_dummy, ax_max, ay_max]
       MatrixXd p_ggv(N, 3);
       for (int i = 0; i < N; ++i) {
           p_ggv(i, 0) = 10.0;
           p_ggv(i, 1) = loc_gg(i, 0);
           p_ggv(i, 2) = loc_gg(i, 1);
       }


       if (params.closed_track)
           return solverFBClosed(p_ggv, ax_max_machines, radii, el_lengths, mu);
       else
           return solverFBUnclosed(p_ggv, ax_max_machines, radii, el_lengths, mu_mean, mu, v_start, v_end);
   }


private:
   // ============================================================
   //  Core: solverFBUnclosed
   // ============================================================
   VectorXd solverFBUnclosed(const MatrixXd& p_ggv,
                             const MatrixXd& ax_max_machines,
                             const VectorXd& radii,
                             const VectorXd& el_lengths,
                             double mu_mean,
                             const VectorXd& mu,
                             double v_start,
                             double v_end)
   {
       int N = radii.size();
       VectorXd vx(N);


       // 초기 곡률 기반 속도
       for (int i = 0; i < N; ++i)
           vx(i) = std::sqrt(std::max(0.0, p_ggv(i, 2) * radii(i)));


       vx = vx.cwiseMin(VectorXd::Constant(N, params.vel_max));
       vx(0) = std::min(vx(0), v_start);


       vx = solverFBAccProfile(p_ggv, ax_max_machines, radii, el_lengths, mu, vx, false);


       vx(N - 1) = std::min(vx(N - 1), v_end);


       vx = solverFBAccProfile(p_ggv, ax_max_machines, radii, el_lengths, mu, vx, true);
       return vx;
   }


   // ============================================================
   //  Core: solverFBClosed
   // ============================================================
   VectorXd solverFBClosed(const MatrixXd& p_ggv,
                           const MatrixXd& ax_max_machines,
                           const VectorXd& radii,
                           const VectorXd& el_lengths,
                           const VectorXd& mu)
   {
       int N = radii.size();
       VectorXd vx(N);


       for (int i = 0; i < N; ++i)
           vx(i) = std::sqrt(std::max(0.0, p_ggv(i, 2) * radii(i)));
       vx = vx.cwiseMin(VectorXd::Constant(N, params.vel_max));


       // 2랩 구성
       VectorXd rad2(2 * N), el2(2 * N), mu2(2 * N), vx2(2 * N);
       for (int i = 0; i < 2 * N; ++i) {
           rad2(i) = radii(i % N);
           el2(i)  = el_lengths(i % el_lengths.size());
           mu2(i)  = mu(i % N);
           vx2(i)  = vx(i % N);
       }


       vx2 = solverFBAccProfile(p_ggv, ax_max_machines, rad2, el2, mu2, vx2, false);
       vx2 = solverFBAccProfile(p_ggv, ax_max_machines, rad2, el2, mu2, vx2, true);


       VectorXd out(N);
       for (int i = 0; i < N; ++i) out(i) = vx2(i + N);
       return out;
   }


   // ============================================================
   //  solverFBAccProfile (forward/backward pass)
   // ============================================================
    VectorXd solverFBAccProfile(const MatrixXd& p_ggv,
                                const MatrixXd& ax_max_machines,
                                const VectorXd& radii,
                                const VectorXd& el_lengths,
                                const VectorXd& mu,
                                VectorXd vx_profile,
                                bool backwards)
    {
        const int Np = static_cast<int>(vx_profile.size());
        VectorXd rad    = radii;
        VectorXd ds     = el_lengths;
        VectorXd vx     = vx_profile;
        VectorXd mu_vec = mu;

        // 열린 트랙: ds.size() == Np-1, 닫힌 트랙/2랩: ds.size() == Np
        const int segs = std::min<int>(ds.size(), std::max(0, Np - 1));

        if (backwards) {
            // 벡터는 전체 뒤집고, 구간 길이는 실제 존재하는 segs만 뒤집기
            rad.reverseInPlace();
            vx.reverseInPlace();
            mu_vec.reverseInPlace();
            if (segs > 0) ds.head(segs).reverseInPlace();
        }

        for (int i = 0; i < segs; ++i) {
            // p_ggv는 보통 N행이므로 모듈로 인덱싱 (닫힌트랙 2랩 대응)
            const int gi = i % static_cast<int>(p_ggv.rows());

            double ax_poss = calcAxPossible(
                vx(i),               // 시작 속도
                rad(i),              // 곡률반경
                p_ggv.row(gi),       // ggv 파라미터(행)
                mu_vec(i),           // 마찰
                backwards            // 전/후진 패스
            );

            // v_{i+1}^2 = v_i^2 + 2 * a_x * ds
            double vx_next_sq = vx(i) * vx(i) + 2.0 * ax_poss * ds(i);
            double vx_next    = std::sqrt(std::max(0.0, vx_next_sq));

            // 현재 상한과 params.vel_max로 모두 클램프
            vx(i + 1) = std::min(vx(i + 1), std::min(vx_next, params.vel_max));
        }

        if (backwards) vx.reverseInPlace();
        return vx;
    }


   // ============================================================
   //  calcAxPossible (타이어 + 모터 + 항력)
   // ============================================================
   double calcAxPossible(double vx_start,
                         double radius,
                         const Eigen::VectorXd& ggv_row,
                         double mu,
                         bool backwards)
   {
       double ax_max_tires = mu * ggv_row(1);
       double ay_max_tires = mu * ggv_row(2);
       double ay_used = vx_start * vx_start / std::max(1e-9, radius);


       double radicand = 1.0 - std::pow(ay_used / ay_max_tires, params.dyn_model_exp);
       double ax_avail_tires = (radicand > 0.0)
                                   ? ax_max_tires * std::pow(radicand, 1.0 / params.dyn_model_exp)
                                   : 0.0;


       // 모터 한계 (단순 보간)
       double ax_machine = params.ax_max;
       if (!backwards) ax_machine *= 0.9;
       double ax_avail_vehicle = std::min(ax_avail_tires, ax_machine);


       // 공기저항
       double ax_drag = -std::pow(vx_start, 2) * params.drag_coeff / params.m_veh;


       if (!backwards)
           return ax_avail_vehicle + ax_drag;
       else
           return ax_avail_vehicle - ax_drag;
   }
};


